"""V3.2 (F4 hardening): targeted cancellation of UNKNOWN-ownership goals by goal ID.

Unknown ownership = the request may have reached Nav2 but the runner holds no goal handle
(send_goal_async raised, or SIGINT landed after it returned but before its future was stored).
V3.1 could only inhibit motion; the goal itself kept executing on the server.

V3.2 chooses the goal ID before sending, so cleanup can cancel exactly that ID through the
action's CancelGoal service and confirm it through the server's status topic. These tests use
a fake *server view* (status per goal id + CancelGoal semantics of rcl_action) on the shared
deterministic clock, and a foreign client's goal that must never be cancelled.
"""
from action_msgs.msg import GoalInfo, GoalStatus
from action_msgs.srv import CancelGoal
import pytest
from rclpy.task import Future

from inspectloop_mission.nav_goal import _uuid, NavGoalRunner
from test_nav_goal import FakeClient, FakeHandle, FakeTime  # noqa: I100 - shared fakes
from test_v3_1_nav_send_interrupt import (  # noqa: I100
    InterruptAfterSend, SendSpyClient, server_goals_quiescent)

ZERO = bytes(16)


class FakeServerView:
    """What the Nav2 action server exposes besides the goal service: status + CancelGoal.

    Goals are known to the server once their send request was answered (accepted ones only;
    rejected goals are not listed, as in rcl_action). Cancel semantics for one goal id:
    unknown -> ERROR_UNKNOWN_GOAL_ID, terminal -> ERROR_GOAL_TERMINATED, else the handle's
    scripted cancel behaviour (ok / reject / never)."""

    def __init__(self, clock, ours, foreign=(), observable=True):
        self.clock, self.ours, self.observable = clock, ours, observable
        self.foreign = list(foreign)
        self.cancel_ids = []           # every goal id a cancel request was sent for

    def _known(self, goal_id):
        key = _uuid(goal_id)
        pairs = list(zip(self.ours.send_futures, self.ours.handles)) + \
            [(f, h) for f, h in self.foreign]
        for f, h in pairs:
            if _uuid(h.goal_id) == key and f.done() and h.accepted:
                return h
        return None

    def status(self, goal_id):
        if not self.observable:
            return None
        h = self._known(goal_id)
        if h is None:
            return None
        if h.result_future.done():
            return h.result_future.result().status
        return GoalStatus.STATUS_CANCELING if h.cancel_calls else GoalStatus.STATUS_EXECUTING

    def cancel(self, goal_id):
        self.cancel_ids.append(_uuid(goal_id))
        h = self._known(goal_id)
        if h is not None and not h.result_future.done():
            return h.cancel_goal_async()              # scripted: ok / reject / never
        f = Future()
        resp = CancelGoal.Response()
        if h is None:
            resp.return_code = CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID
        else:
            resp.return_code = CancelGoal.Response.ERROR_GOAL_TERMINATED
            gi = GoalInfo()
            gi.goal_id = h.goal_id
            resp.goals_canceling = [gi]
        self.clock.at(0.2, lambda: f.set_result(resp))
        return f


class Motion:
    def __init__(self, stopped=True):
        self.stopped, self.calls = stopped, 0

    def __call__(self):
        self.calls += 1
        return self.stopped


def make(c, scripts, observable=True, motion=None, foreign=(), with_fallback=True):
    FakeHandle._n = 0
    cl = SendSpyClient(c, scripts)
    view = FakeServerView(c, cl, foreign, observable)
    motion = motion if motion is not None else Motion()
    r = NavGoalRunner(cl, c.wait, lambda: c.t, None, accept_timeout_s=2.0, cancel_timeout_s=2.0,
                      terminal_timeout_s=2.0, goal_id_fallback=view if with_fallback else None,
                      motion_stopped=motion)
    return r, cl, view, motion


def lose_future(r, cl):
    """SIGINT after send_goal_async returned, before its future was stored."""
    with InterruptAfterSend(cl, nth=1, mode='opcode') as inj:
        with pytest.raises(KeyboardInterrupt):
            r.run('g', 'A', 30.0)
    assert inj.where.startswith('run:')
    assert any('ownership unknown' in u for u in r.unresolved())


def foreign_goal(c):
    """A goal of another client (RViz panel / waypoint follower) executing on the server."""
    other = FakeClient(c, [{'answer_after': 0.1, 'result_after': None, 'cancel': 'ok'}])
    f = other.send_goal_async('g', goal_uuid=None)
    c.advance(0.2)
    h = other.handles[0]
    assert f.done() and h.accepted
    return f, h


# --------------------------------------------------------------------- the main regression
def test_lost_future_goal_is_cancelled_by_its_id_and_confirmed_terminal():
    c = FakeTime()
    r, cl, view, motion = make(c, [{'answer_after': 0.5, 'result_after': None, 'cancel': 'ok'}])
    lose_future(r, cl)                         # cleanup inside run(): server did not know it yet
    h = cl.handles[0]
    assert r.cancel_all('shutdown') is False   # nothing confirmed yet
    c.advance(1.0)                             # the server accepts the request: robot would drive
    assert h.accepted and not h.result_future.done()
    assert r.motion_inhibited()                # status shows our id active -> cancel requested
    assert h.cancel_calls == 1
    assert r.cancel_all('shutdown') is True    # ERROR_NONE + server status CANCELED + stopped
    assert h.result_future.result().status == GoalStatus.STATUS_CANCELED
    assert not r.motion_inhibited()
    assert motion.calls >= 1
    assert set(view.cancel_ids) == {_uuid(h.goal_id)} and ZERO not in view.cancel_ids
    kinds = [e['event'] for e in r.events]
    assert 'UNKNOWN_GOAL_ACTIVE' in kinds and 'UNKNOWN_RESOLVED' in kinds


def test_goal_ids_are_chosen_before_sending_and_used_by_the_server():
    c = FakeTime()
    ids = []

    def new_id():
        from inspectloop_mission.nav_goal import random_goal_id
        ids.append(random_goal_id())
        return ids[-1]
    FakeHandle._n = 0
    cl = SendSpyClient(c, [{'answer_after': 0.2, 'result_after': 1.0}])
    r = NavGoalRunner(cl, c.wait, lambda: c.t, None, accept_timeout_s=2.0, new_goal_id=new_id)
    assert r.run('g', 'A', 5.0)['result'] == 'SUCCEEDED'
    assert len(ids) == 1 and _uuid(cl.handles[0].goal_id) == _uuid(ids[0])
    assert _uuid(ids[0]) != ZERO


# ------------------------------------------------------------- never cancel other goals
def test_only_our_goal_id_is_cancelled_never_a_foreign_goal():
    c = FakeTime()
    FakeHandle._n = 100                         # distinct ids for the foreign client
    ff, fh = foreign_goal(c)
    r, cl, view, _ = make(c, [{'answer_after': 0.5, 'result_after': None, 'cancel': 'ok'}],
                          foreign=[(ff, fh)])
    lose_future(r, cl)
    c.advance(1.0)
    assert r.cancel_all('shutdown') is True
    assert fh.cancel_calls == 0 and not fh.result_future.done()   # foreign goal untouched
    assert view.cancel_ids and all(i == _uuid(cl.handles[0].goal_id) for i in view.cancel_ids)
    assert ZERO not in view.cancel_ids


# ----------------------------------------------------------------- fail-closed outcomes
def test_request_never_reaching_the_server_stays_unresolved():
    """ERROR_UNKNOWN_GOAL_ID cannot distinguish 'lost' from 'still in flight': fail closed."""
    c = FakeTime()
    r, cl, view, _ = make(c, [{'answer_after': None}])
    lose_future(r, cl)
    for _ in range(4):
        c.advance(3.0)
        assert r.cancel_all('shutdown') is False
        assert r.motion_inhibited()
    assert r.run('g', 'B', 5.0)['result'] == 'INHIBITED'
    assert len(cl.handles) == 1
    assert any('ERROR_UNKNOWN_GOAL_ID' in e['detail'] or 'does not know' in e['detail']
               for e in r.events if e['event'] == 'UNKNOWN_UNRESOLVED')


def test_cancel_sent_but_status_unobservable_is_not_success():
    c = FakeTime()
    r, cl, view, _ = make(c, [{'answer_after': 0.5, 'result_after': None, 'cancel': 'ok'}],
                          observable=False)
    lose_future(r, cl)
    c.advance(1.0)
    ok = r.cancel_all('shutdown')
    assert view.cancel_ids                      # a cancel request WAS sent ...
    assert ok is False                          # ... but terminal status was never observed
    assert r.motion_inhibited()


@pytest.mark.parametrize('mode', ['reject', 'never'])
def test_cancel_refused_or_unanswered_stays_unresolved(mode):
    c = FakeTime()
    r, cl, _, _ = make(c, [{'answer_after': 0.5, 'result_after': None, 'cancel': mode}])
    lose_future(r, cl)
    c.advance(1.0)
    for _ in range(3):
        assert r.cancel_all('shutdown') is False
        c.advance(1.0)
    assert r.motion_inhibited()
    assert not cl.handles[0].result_future.done()


def test_cancel_accepted_but_goal_never_terminal_stays_unresolved():
    c = FakeTime()
    r, cl, _, _ = make(c, [{'answer_after': 0.5, 'result_after': None, 'cancel': 'ok',
                            'terminal_after_cancel': None}])
    lose_future(r, cl)
    c.advance(1.0)
    assert r.cancel_all('shutdown') is False
    assert r.motion_inhibited()


def test_robot_not_stationary_after_resolution_is_not_clean():
    c = FakeTime()
    motion = Motion(stopped=False)
    r, cl, _, _ = make(c, [{'answer_after': 0.5, 'result_after': None, 'cancel': 'ok'}],
                       motion=motion)
    lose_future(r, cl)
    c.advance(1.0)
    assert r.cancel_all('shutdown') is False    # goal CANCELED but motion not confirmed safe
    assert any(e['event'] == 'MOTION_NOT_CONFIRMED_STOPPED' for e in r.events)
    motion.stopped = True
    assert r.cancel_all('shutdown') is True


def test_goal_already_terminal_on_the_server_resolves_without_cancelling():
    c = FakeTime()
    r, cl, view, _ = make(c, [{'answer_after': 0.5, 'result_after': 0.8}])
    lose_future(r, cl)
    c.advance(2.0)                              # it succeeded on its own
    assert r.cancel_all('shutdown') is True
    assert cl.handles[0].cancel_calls == 0


def test_without_fallback_unknown_ownership_still_inhibits_permanently():
    c = FakeTime()
    r, cl, _, _ = make(c, [{'answer_after': 0.5, 'result_after': None, 'cancel': 'ok'}],
                       with_fallback=False)
    lose_future(r, cl)
    c.advance(5.0)
    assert r.cancel_all('shutdown') is False
    assert r.run('g', 'B', 5.0)['result'] == 'INHIBITED'


def test_fallback_exceptions_and_second_interrupt_never_escape_cleanup():
    c = FakeTime()
    r, cl, view, _ = make(c, [{'answer_after': 0.5, 'result_after': None, 'cancel': 'ok'}])
    lose_future(r, cl)
    c.advance(1.0)

    def boom(goal_id):
        raise RuntimeError('cancel transport failed')
    view.cancel = boom
    r._ownership_unknown[0].cancel_future = None
    assert r.cancel_all('shutdown') is False
    real_status = view.status
    view.status = lambda goal_id: (_ for _ in ()).throw(KeyboardInterrupt('2nd ctrl-c'))
    assert r.cancel_all('shutdown') is False    # never raises, never reports clean
    view.status = real_status
    assert r.motion_inhibited()
    assert any(e['event'] == 'CLEANUP_EXCEPTION' for e in r.events)


def test_send_goal_raising_after_transmission_is_cancelled_by_id():
    """send_goal_async raises although the request reached the server."""
    c = FakeTime()

    class RaiseAfterSend(SendSpyClient):
        def send_goal_async(self, goal, feedback_callback=None, **kw):
            super().send_goal_async(goal, feedback_callback, **kw)
            raise RuntimeError('rcl error after publishing the request')
    FakeHandle._n = 0
    cl = RaiseAfterSend(c, [{'answer_after': 0.5, 'result_after': None, 'cancel': 'ok'}])
    view = FakeServerView(c, cl)
    r = NavGoalRunner(cl, c.wait, lambda: c.t, None, accept_timeout_s=2.0, cancel_timeout_s=2.0,
                      terminal_timeout_s=2.0, goal_id_fallback=view, motion_stopped=Motion())
    with pytest.raises(RuntimeError):
        r.run('g', 'A', 5.0)
    c.advance(1.0)
    assert r.cancel_all('shutdown') is True
    assert cl.handles[0].result_future.result().status == GoalStatus.STATUS_CANCELED


def test_interrupt_before_sending_does_not_inhibit():
    c = FakeTime()

    def interrupted_id():
        raise KeyboardInterrupt('before the request existed')
    cl = SendSpyClient(c, [{'answer_after': 0.2, 'result_after': 1.0}])
    r = NavGoalRunner(cl, c.wait, lambda: c.t, None, new_goal_id=interrupted_id)
    with pytest.raises(KeyboardInterrupt):
        r.run('g', 'A', 5.0)
    assert not cl.handles and not r.motion_inhibited()
    assert r.cancel_all('shutdown') is True


@pytest.mark.parametrize('script,resolvable', [
    ({'answer_after': 0.5, 'result_after': None, 'cancel': 'ok'}, True),
    ({'answer_after': 3.0, 'result_after': None, 'cancel': 'ok'}, True),
    ({'answer_after': 0.5, 'result_after': None, 'cancel': 'reject'}, False),
])
def test_sigint_at_every_opcode_with_fallback(script, resolvable):
    """Every interruption point after the send: never a false clean shutdown, and with a
    cooperative server every point ends resolved (no permanently stranded goal)."""
    fired = 0
    for nth in range(1, 5000):
        c = FakeTime()
        r, cl, view, _ = make(c, [dict(script)])
        with InterruptAfterSend(cl, nth=nth, mode='opcode') as inj:
            try:
                r.run('g', 'A', 4.0)
            except KeyboardInterrupt:
                pass
        if not inj.fired:
            break
        fired += 1
        ok = False
        for _ in range(8):
            c.advance(1.0)
            ok = r.cancel_all('shutdown')
            assert not (ok and not server_goals_quiescent(cl)), f'false clean at {inj.where}'
            assert ok or r.motion_inhibited()
        if resolvable:
            assert ok and server_goals_quiescent(cl), f'stranded goal after SIGINT at {inj.where}'
        assert ZERO not in view.cancel_ids
    else:
        pytest.fail('sweep did not reach the end of run()')
    assert fired > 20
