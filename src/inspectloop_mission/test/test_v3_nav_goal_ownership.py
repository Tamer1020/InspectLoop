"""V3 (Astra V2 review, F4 HIGH): goal ownership across the send -> acceptance window.

The V2 runner only recorded the send future after the acceptance wait timed out; an exception
or KeyboardInterrupt raised *inside* that wait lost the future, so a goal accepted afterwards
was owned by nobody: cancel_all() reported success, nothing was cancelled and the robot drove.
"""
from action_msgs.msg import GoalStatus
import pytest

from inspectloop_mission.nav_goal import NavGoalRunner
from test_nav_goal import FakeClient, FakeTime  # noqa: I100 - shared deterministic fakes


class InterruptingWait:
    """clock.wait that raises `exc` the first time it is asked to wait for `target()`."""

    def __init__(self, clock, exc, target):
        self.clock, self.exc, self.target, self.fired = clock, exc, target, False

    def __call__(self, fut, timeout, abort_check=None):
        if not self.fired and fut is self.target():
            self.fired = True
            self.clock.advance(0.1)            # part-way through the acceptance wait
            raise self.exc
        return self.clock.wait(fut, timeout, abort_check)


class SpyClient(FakeClient):
    """Records the send future so the test can aim the interruption at the acceptance wait."""

    def __init__(self, clock, scripts):
        super().__init__(clock, scripts)
        self.send_futures = []

    def send_goal_async(self, goal, feedback_callback=None):
        f = super().send_goal_async(goal, feedback_callback)
        self.send_futures.append(f)
        return f


def make(clock, scripts, exc):
    cl = SpyClient(clock, scripts)
    w = InterruptingWait(clock, exc, lambda: cl.send_futures[-1] if cl.send_futures else None)
    r = NavGoalRunner(cl, w, lambda: clock.t, None, accept_timeout_s=2.0, cancel_timeout_s=2.0,
                      terminal_timeout_s=2.0)
    return r, cl


@pytest.mark.parametrize('exc', [KeyboardInterrupt(), RuntimeError('boom')])
def test_interrupt_during_acceptance_then_late_acceptance_is_owned_and_cancelled(exc):
    """The mandatory regression: interrupt inside the acceptance wait, acceptance arrives late
    (after the runner's bounded cleanup gave up waiting for it)."""
    c = FakeTime()
    r, cl = make(c, [{'answer_after': 5.0, 'result_after': None, 'cancel': 'ok'}], exc)
    with pytest.raises(type(exc)):
        r.run('g', 'A', 30.0)
    # the request is still in flight: it is owned, unresolved and inhibits motion
    assert r.unresolved() == ['A']
    assert r.motion_inhibited()
    assert r.cancel_all('shutdown') is False    # cannot claim a clean shutdown yet
    c.advance(5.0)                              # goal accepted after the interruption
    h = cl.handles[0]
    assert r.motion_inhibited()                 # late acceptance -> cancel requested, unconfirmed
    assert h.cancel_calls == 1                  # cancellation actually requested
    ok = r.cancel_all('shutdown')
    assert h.result_future.done() and h.result_future.result().status == \
        GoalStatus.STATUS_CANCELED              # terminal confirmed
    assert ok is True
    assert not r.motion_inhibited()


@pytest.mark.parametrize('exc', [KeyboardInterrupt(), RuntimeError('boom')])
def test_interrupt_during_acceptance_acceptance_within_cleanup_window_is_cancelled(exc):
    """Acceptance arriving while the exception cleanup still waits (bounded) is cancelled and
    confirmed before the exception propagates."""
    c = FakeTime()
    r, cl = make(c, [{'answer_after': 1.0, 'result_after': None, 'cancel': 'ok'}], exc)
    with pytest.raises(type(exc)):
        r.run('g', 'A', 30.0)
    h = cl.handles[0]
    assert h.cancel_calls == 1
    assert h.result_future.result().status == GoalStatus.STATUS_CANCELED
    assert not r.motion_inhibited()
    assert any(e['event'] == 'LATE_ACCEPTANCE' for e in r.events)
    assert any(e['event'] == 'CANCEL_CONFIRMED' for e in r.events)


def test_interrupt_during_acceptance_never_answered_cannot_report_clean_shutdown():
    c = FakeTime()
    r, cl = make(c, [{'answer_after': None}], KeyboardInterrupt())
    with pytest.raises(KeyboardInterrupt):
        r.run('g', 'A', 30.0)
    assert r.cancel_all('shutdown') is False
    assert r.motion_inhibited()
    assert r.unresolved() == ['A']


def test_interrupt_during_acceptance_late_goal_whose_cancel_fails_stays_inhibited():
    c = FakeTime()
    r, cl = make(c, [{'answer_after': 1.0, 'result_after': None, 'cancel': 'reject'}],
                 KeyboardInterrupt())
    with pytest.raises(KeyboardInterrupt):
        r.run('g', 'A', 30.0)
    c.advance(1.0)
    assert r.cancel_all('shutdown') is False
    assert cl.handles[0].cancel_calls >= 1
    assert r.motion_inhibited()


def test_no_new_goal_is_dispatched_while_the_interrupted_goal_is_unresolved():
    c = FakeTime()
    r, cl = make(c, [{'answer_after': None}, {'answer_after': 0.2, 'result_after': 1.0}],
                 KeyboardInterrupt())
    with pytest.raises(KeyboardInterrupt):
        r.run('g', 'A', 30.0)
    out = r.run('g', 'B', 30.0)
    assert out['result'] == 'INHIBITED'
    assert len(cl.handles) == 1             # B was never sent


class ExplodingHandleClient(FakeClient):
    """Accepted goal whose cancel_goal_async raises (cleanup itself fails)."""

    def send_goal_async(self, goal, feedback_callback=None):
        f = super().send_goal_async(goal, feedback_callback)
        h = self.handles[-1]

        def boom():
            h.cancel_calls += 1
            raise RuntimeError('cancel transport failed')
        h.cancel_goal_async = boom
        return f


def test_exception_thrown_by_cleanup_itself_keeps_motion_inhibited():
    c = FakeTime()
    cl = ExplodingHandleClient(c, [{'answer_after': 0.2, 'result_after': None}])
    r = NavGoalRunner(cl, c.wait, lambda: c.t, None, accept_timeout_s=2.0, cancel_timeout_s=2.0,
                      terminal_timeout_s=2.0)
    out = r.run('g', 'A', 3.0)                  # times out -> cleanup -> cancel raises
    assert out['result'] == 'TIMEOUT' and out['cancel_confirmed'] is False
    assert r.cancel_all('shutdown') is False    # never reports success, never raises
    assert r.motion_inhibited()
    assert any(e['event'] == 'CLEANUP_EXCEPTION' for e in r.events)
    assert r.run('g', 'B', 3.0)['result'] == 'INHIBITED'


def test_second_interrupt_during_cleanup_does_not_escape_cancel_all():
    c = FakeTime()
    cl = FakeClient(c, [{'answer_after': 0.2, 'result_after': None, 'cancel': 'ok'}])
    calls = {'n': 0}

    def wait(fut, timeout, abort_check=None):
        calls['n'] += 1
        if calls['n'] == 3:                     # 1 accept, 2 result (times out), 3 cancel answer
            raise KeyboardInterrupt('second ctrl-c')
        return c.wait(fut, timeout, abort_check)
    r = NavGoalRunner(cl, wait, lambda: c.t, None, accept_timeout_s=2.0, cancel_timeout_s=2.0,
                      terminal_timeout_s=2.0)
    out = r.run('g', 'A', 1.0)
    assert out['result'] == 'TIMEOUT' and out['cancel_confirmed'] is False
    assert r.motion_inhibited()                 # still owned; a later cancel_all can resolve it
    assert r.cancel_all('retry') is True
    assert not r.motion_inhibited()


def test_send_goal_raising_makes_ownership_unknown_and_inhibits_permanently():
    c = FakeTime()

    class BadClient(FakeClient):
        def send_goal_async(self, goal, feedback_callback=None):
            raise RuntimeError('rcl send failed after publishing?')
    r = NavGoalRunner(BadClient(c, []), c.wait, lambda: c.t, None)
    with pytest.raises(RuntimeError):
        r.run('g', 'A', 3.0)
    assert r.motion_inhibited()
    assert r.cancel_all('shutdown') is False
    assert r.run('g', 'B', 3.0)['result'] == 'INHIBITED'


def test_timeout_rejection_and_success_paths_leave_nothing_owned():
    c = FakeTime()
    cl = FakeClient(c, [{'answer_after': 0.2, 'accept': False},
                        {'answer_after': 0.2, 'result_after': 1.0},
                        {'answer_after': 0.2, 'result_after': None, 'cancel': 'ok'}])
    r = NavGoalRunner(cl, c.wait, lambda: c.t, None, accept_timeout_s=2.0, cancel_timeout_s=2.0,
                      terminal_timeout_s=2.0)
    assert r.run('g', 'A', 5.0)['result'] == 'REJECTED'
    assert not r.motion_inhibited()
    assert r.run('g', 'B', 5.0)['result'] == 'SUCCEEDED'
    assert not r.motion_inhibited()
    assert r.run('g', 'C', 1.0)['result'] == 'TIMEOUT'
    assert not r.motion_inhibited()
    assert r.cancel_all('end') is True
