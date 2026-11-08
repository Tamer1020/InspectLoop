"""Astra F4: Nav2 goal lifecycle - deterministic tests with a scripted fake action client."""
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
import pytest
from rclpy.task import Future
from unique_identifier_msgs.msg import UUID

from inspectloop_mission.nav_goal import NavGoalRunner


class FakeTime:
    def __init__(self):
        self.t = 0.0
        self.events = []          # (time, fn)

    def at(self, dt, fn):
        self.events.append((self.t + dt, fn))

    def advance(self, dt):
        self.t += dt
        due = sorted([e for e in self.events if e[0] <= self.t], key=lambda e: e[0])
        self.events = [e for e in self.events if e[0] > self.t]
        for _, fn in due:
            fn()

    def wait(self, fut, timeout, abort_check=None):
        end = self.t + timeout
        while not fut.done() and self.t < end:
            self.advance(0.1)
            if abort_check is not None:
                abort_check()
        return fut.done()


def _result(status):
    return SimpleNamespace(status=status, result=SimpleNamespace(error_code=0, error_msg=''))


class FakeHandle:
    _n = 0

    def __init__(self, clock, accepted, script):
        FakeHandle._n += 1
        self.accepted = accepted
        self.goal_id = UUID(uuid=[FakeHandle._n] * 16)
        self.clock, self.script = clock, script
        self.result_future = Future()
        self.cancel_calls = 0
        if accepted and script.get('result_after') is not None:
            clock.at(script['result_after'],
                     lambda: self.result_future.done() or self.result_future.set_result(
                         _result(script.get('result_status', GoalStatus.STATUS_SUCCEEDED))))

    def get_result_async(self):
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        f = Future()
        mode = self.script.get('cancel', 'ok')
        if mode == 'never':
            return f
        resp = CancelGoal.Response()
        if mode == 'ok':
            resp.return_code = CancelGoal.Response.ERROR_NONE
            info = CancelGoal.Response().goals_canceling
            from action_msgs.msg import GoalInfo
            gi = GoalInfo()
            gi.goal_id = self.goal_id
            resp.goals_canceling = [gi]
            if self.script.get('terminal_after_cancel', 0.3) is not None:
                self.clock.at(self.script.get('terminal_after_cancel', 0.3),
                              lambda: self.result_future.done() or self.result_future.set_result(
                                  _result(GoalStatus.STATUS_CANCELED)))
        elif mode == 'reject':
            resp.return_code = CancelGoal.Response.ERROR_REJECTED
        self.clock.at(self.script.get('cancel_answer_after', 0.2), lambda: f.set_result(resp))
        return f


class FakeClient:
    def __init__(self, clock, scripts):
        self.clock, self.scripts, self.handles = clock, list(scripts), []

    def send_goal_async(self, goal, feedback_callback=None):
        script = self.scripts.pop(0)
        f = Future()
        h = FakeHandle(self.clock, script.get('accept', True), script)
        self.handles.append(h)
        if script.get('answer_after') is not None:
            self.clock.at(script['answer_after'], lambda: f.set_result(h))
        return f


def runner(clock, client, **kw):
    return NavGoalRunner(client, clock.wait, lambda: clock.t, None, accept_timeout_s=2.0,
                         cancel_timeout_s=2.0, terminal_timeout_s=2.0, **kw)


def test_success():
    c = FakeTime()
    cl = FakeClient(c, [{'answer_after': 0.2, 'result_after': 3.0}])
    r = runner(c, cl)
    assert r.run('g', 'A', 10.0)['result'] == 'SUCCEEDED'
    assert not r.motion_inhibited()


def test_rejected_goal():
    c = FakeTime()
    r = runner(c, FakeClient(c, [{'answer_after': 0.2, 'accept': False}]))
    assert r.run('g', 'A', 10.0)['result'] == 'REJECTED'
    assert not r.motion_inhibited()


def test_timeout_cancels_and_confirms_terminal():
    c = FakeTime()
    cl = FakeClient(c, [{'answer_after': 0.2, 'result_after': None, 'cancel': 'ok'}])
    r = runner(c, cl)
    out = r.run('g', 'A', 5.0)
    assert out['result'] == 'TIMEOUT' and out['cancel_confirmed'] is True
    assert cl.handles[0].cancel_calls == 1
    assert not r.motion_inhibited()


def test_late_acceptance_is_cancelled_and_blocks_motion_until_resolved():
    c = FakeTime()
    # answered after the 2 s accept timeout; goal would run for a long time
    cl = FakeClient(c, [{'answer_after': 3.0, 'result_after': None, 'cancel': 'ok',
                         'terminal_after_cancel': 0.5},
                        {'answer_after': 0.2, 'result_after': 1.0}])
    r = runner(c, cl)
    assert r.run('g', 'A', 10.0)['result'] == 'NO_RESPONSE'
    assert r.motion_inhibited()                 # request may still be accepted
    c.advance(1.5)                              # late acceptance arrives
    assert r.motion_inhibited()                 # cancelled but not yet terminal
    assert cl.handles[0].cancel_calls == 1
    out = r.run('g2', 'B', 10.0)                # waits for confirmation, then proceeds
    assert out['result'] == 'SUCCEEDED'
    assert any(e['event'] == 'LATE_ACCEPTANCE' for e in r.events)


def test_failed_cancellation_inhibits_new_motion():
    c = FakeTime()
    cl = FakeClient(c, [{'answer_after': 0.2, 'result_after': None, 'cancel': 'reject'},
                        {'answer_after': 0.2, 'result_after': 1.0}])
    r = runner(c, cl)
    out = r.run('g', 'A', 5.0)
    assert out['result'] == 'TIMEOUT' and out['cancel_confirmed'] is False
    assert r.motion_inhibited()
    out2 = r.run('g2', 'B', 10.0)
    assert out2['result'] == 'INHIBITED'
    assert len(cl.handles) == 1                 # no second goal was sent


def test_cancel_never_answered_inhibits():
    c = FakeTime()
    r = runner(c, FakeClient(c, [{'answer_after': 0.2, 'result_after': None, 'cancel': 'never'}]))
    assert r.run('g', 'A', 3.0)['cancel_confirmed'] is False
    assert r.motion_inhibited()


def test_cancel_accepted_but_no_terminal_result_inhibits():
    c = FakeTime()
    r = runner(c, FakeClient(c, [{'answer_after': 0.2, 'result_after': None, 'cancel': 'ok',
                                  'terminal_after_cancel': None}]))
    out = r.run('g', 'A', 3.0)
    assert out['cancel_confirmed'] is False and r.motion_inhibited()


class Boom(Exception):
    pass


def test_exception_while_waiting_cancels_goal():
    c = FakeTime()
    cl = FakeClient(c, [{'answer_after': 0.2, 'result_after': None, 'cancel': 'ok'}])
    r = runner(c, cl)
    calls = {'n': 0}

    def abort_check():
        calls['n'] += 1
        if calls['n'] > 5:
            raise Boom('lidar failed')
    with pytest.raises(Boom):
        r.run('g', 'A', 30.0, abort_check)
    assert cl.handles[0].cancel_calls == 1
    assert not r.motion_inhibited()
    assert any(e['event'] == 'CANCEL_CONFIRMED' for e in r.events)


def test_keyboard_interrupt_cancels_goal():
    c = FakeTime()
    cl = FakeClient(c, [{'answer_after': 0.2, 'result_after': None, 'cancel': 'ok'}])
    r = runner(c, cl)

    def interrupt():
        if c.t > 1.0:
            raise KeyboardInterrupt
    with pytest.raises(KeyboardInterrupt):
        r.run('g', 'A', 30.0, interrupt)
    assert cl.handles[0].cancel_calls == 1


def test_cancel_all_on_shutdown_handles_pending_late_acceptance():
    c = FakeTime()
    cl = FakeClient(c, [{'answer_after': 2.5, 'result_after': None, 'cancel': 'ok'}])
    r = runner(c, cl)
    assert r.run('g', 'A', 10.0)['result'] == 'NO_RESPONSE'
    assert r.cancel_all('shutdown') is True
    assert cl.handles[0].cancel_calls == 1 and not r.motion_inhibited()


def test_goal_that_finishes_while_cancel_refused_is_resolved():
    """Cancel refused because the goal already finished -> resolved, motion allowed."""
    c = FakeTime()
    cl = FakeClient(c, [{'answer_after': 0.2, 'result_after': 3.3, 'cancel': 'reject',
                         'cancel_answer_after': 0.5},
                        {'answer_after': 0.2, 'result_after': 1.0}])
    r = runner(c, cl)
    out = r.run('g', 'A', 3.0)
    assert out['result'] == 'TIMEOUT' and out['cancel_confirmed'] is True
    assert r.run('g2', 'B', 5.0)['result'] == 'SUCCEEDED'
