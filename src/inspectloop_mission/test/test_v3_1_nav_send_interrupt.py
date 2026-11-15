"""V3.1 (audit F4 HIGH): interruption between `send_goal_async()` returning and registration.

V3 registered the pending send on the line *after* `send_goal_async()` returned. A SIGINT
(KeyboardInterrupt) delivered in that gap propagated out of `run()` with the request sent but
owned by nobody: `cancel_all()` then reported a clean shutdown while Nav2 accepted and executed
the goal.

The interruption is injected deterministically with `sys.settrace`: the tracer raises
KeyboardInterrupt from inside the runner's frame at the first trace event after the (fake)
action client's `send_goal_async()` has returned - exactly where CPython would deliver a
pending SIGINT. An exception raised by a trace function propagates into the traced frame.
"""
import inspect
import sys

from action_msgs.msg import GoalStatus
import pytest

from inspectloop_mission import nav_goal
from inspectloop_mission.nav_goal import NavGoalRunner
from test_nav_goal import FakeClient, FakeHandle, FakeTime  # noqa: I100 - shared fakes

NAV_GOAL_FILE = nav_goal.__file__


class SendSpyClient(FakeClient):
    def __init__(self, clock, scripts):
        super().__init__(clock, scripts)
        self.returned = False
        self.send_futures = []

    def send_goal_async(self, goal, feedback_callback=None, **kw):
        f = super().send_goal_async(goal, feedback_callback, **kw)
        self.send_futures.append(f)
        self.returned = True          # the request now exists on the (fake) server
        return f


class InterruptAfterSend:
    """Raise KeyboardInterrupt at the `nth` trace event inside nav_goal.py once
    `send_goal_async()` has returned. mode='line': line events; 'opcode': every bytecode."""

    def __init__(self, client, nth=1, mode='line'):
        self.client, self.nth, self.mode = client, nth, mode
        self.count = 0
        self.fired = False
        self.where = None

    def _local(self, frame, event, arg):
        if self.fired or not self.client.returned or event != self.mode:
            return self._local
        if frame.f_code.co_flags & inspect.CO_GENERATOR:
            return self._local        # an exception raised while a generator closes is unraisable
        self.count += 1
        if self.count == self.nth:
            self.fired = True
            self.where = f'{frame.f_code.co_name}:{frame.f_lineno}'
            sys.settrace(None)
            raise KeyboardInterrupt(f'SIGINT injected at {self.where}')
        return self._local

    def _global(self, frame, event, arg):
        if frame.f_code.co_filename != NAV_GOAL_FILE or self.fired:
            return None
        if self.mode == 'opcode':
            frame.f_trace_opcodes = True
            frame.f_trace = self._local
            sys.settrace(self._global)      # CPython 3.12: re-arm so opcode events take effect
        return self._local

    def __enter__(self):
        sys.settrace(self._global)
        return self

    def __exit__(self, *exc):
        sys.settrace(None)
        return False


def make(clock, scripts):
    FakeHandle._n = 0                 # fake goal-id bytes must stay within uint8
    cl = SendSpyClient(clock, scripts)
    r = NavGoalRunner(cl, clock.wait, lambda: clock.t, None, accept_timeout_s=2.0,
                      cancel_timeout_s=2.0, terminal_timeout_s=2.0)
    return r, cl


def server_goals_quiescent(cl) -> bool:
    """True when no request sent to the fake server can still move the robot."""
    for f, h in zip(cl.send_futures, cl.handles):
        if not f.done():
            return False              # will be answered (possibly accepted) later
        if h.accepted and not h.result_future.done():
            return False              # accepted and still executing
    return True


# ---------------------------------------------------------------- the mandatory regression
def test_sigint_right_after_send_goal_async_returns_is_owned_and_cancelled():
    """Interrupt on the first line executed after send_goal_async() returns (the V3
    registration point). Acceptance arrives late, after the bounded exception cleanup."""
    c = FakeTime()
    r, cl = make(c, [{'answer_after': 5.0, 'result_after': None, 'cancel': 'ok'}])
    with InterruptAfterSend(cl, nth=1, mode='line') as inj:
        with pytest.raises(KeyboardInterrupt):
            r.run('g', 'A', 30.0)
    assert inj.fired and inj.where.startswith('run:')
    # the request exists on the server: it must be owned, unresolved and inhibit motion
    assert r.motion_inhibited()
    assert r.unresolved() == ['A']
    assert r.cancel_all('shutdown') is False     # no clean shutdown while ownership is open
    c.advance(5.0)                               # late acceptance
    h = cl.handles[0]
    assert r.motion_inhibited()                  # polled: cancel requested, not yet terminal
    assert h.accepted and h.cancel_calls == 1    # cancelled as soon as it was seen
    assert r.cancel_all('shutdown') is True
    assert h.result_future.result().status == GoalStatus.STATUS_CANCELED
    assert not r.motion_inhibited()
    assert any(e['event'] == 'LATE_ACCEPTANCE' for e in r.events)


def test_sigint_right_after_send_acceptance_inside_cleanup_window():
    c = FakeTime()
    r, cl = make(c, [{'answer_after': 0.5, 'result_after': None, 'cancel': 'ok'}])
    with InterruptAfterSend(cl, nth=1, mode='line'):
        with pytest.raises(KeyboardInterrupt):
            r.run('g', 'A', 30.0)
    h = cl.handles[0]
    assert h.cancel_calls == 1
    assert h.result_future.result().status == GoalStatus.STATUS_CANCELED
    assert not r.motion_inhibited()
    assert server_goals_quiescent(cl)


def test_sigint_right_after_send_never_answered_never_reports_clean_shutdown():
    c = FakeTime()
    r, cl = make(c, [{'answer_after': None}])
    with InterruptAfterSend(cl, nth=1, mode='line'):
        with pytest.raises(KeyboardInterrupt):
            r.run('g', 'A', 30.0)
    for _ in range(5):
        c.advance(3.0)
        assert r.cancel_all('shutdown') is False
        assert r.motion_inhibited()
    assert r.run('g', 'B', 5.0)['result'] == 'INHIBITED'
    assert len(cl.handles) == 1                  # nothing else was dispatched


def test_sigint_before_future_is_stored_makes_ownership_unknown():
    """Opcode-level: the interrupt lands after the CALL returned but before `pend.future` is
    stored. The future is lost, so ownership is unknown: permanently inhibited, never clean."""
    c = FakeTime()
    r, cl = make(c, [{'answer_after': 0.5, 'result_after': None, 'cancel': 'ok'}])
    with InterruptAfterSend(cl, nth=1, mode='opcode') as inj:
        with pytest.raises(KeyboardInterrupt):
            r.run('g', 'A', 30.0)
    assert inj.where.startswith('run:')
    c.advance(5.0)
    assert r.cancel_all('shutdown') is False
    assert r.motion_inhibited()
    assert any('ownership unknown' in u for u in r.unresolved())
    assert r.run('g', 'B', 5.0)['result'] == 'INHIBITED'


@pytest.mark.parametrize('script', [
    {'answer_after': 0.5, 'result_after': None, 'cancel': 'ok'},
    {'answer_after': 0.5, 'result_after': None, 'cancel': 'reject'},
    {'answer_after': 0.5, 'result_after': None, 'cancel': 'never'},
    {'answer_after': 3.0, 'result_after': None, 'cancel': 'ok'},
    {'answer_after': 0.5, 'accept': False},
])
def test_sigint_at_every_opcode_after_send_never_yields_false_clean_shutdown(script):
    """Sweep the interruption over every bytecode executed in nav_goal.py after the send
    returned (send -> accept -> track -> result wait). Invariant at every point: cleanup may
    only report success when no sent request can still move the robot, and while it reports
    failure motion stays inhibited."""
    fired_points = 0
    for nth in range(1, 5000):
        c = FakeTime()
        r, cl = make(c, [dict(script)])
        with InterruptAfterSend(cl, nth=nth, mode='opcode') as inj:
            try:
                r.run('g', 'A', 4.0)
            except KeyboardInterrupt:
                pass
        if not inj.fired:
            break                                 # run() completed: every point covered
        fired_points += 1
        for _ in range(6):
            c.advance(1.0)
            ok = r.cancel_all('shutdown')
            assert not (ok and not server_goals_quiescent(cl)), \
                f'false clean shutdown after SIGINT at {inj.where} (#{nth})'
            assert ok or r.motion_inhibited(), f'unresolved but not inhibited at {inj.where}'
    else:
        pytest.fail('sweep did not reach the end of run()')
    assert fired_points > 20
