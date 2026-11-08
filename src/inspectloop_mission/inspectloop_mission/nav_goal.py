"""Nav2 NavigateToPose goal lifecycle with verified cancellation (Astra F4).

Invariants
----------
* At most one goal of ours may be able to move the robot. Every goal handle is tracked until a
  *terminal* result (SUCCEEDED / ABORTED / CANCELED) has been observed.
* A goal request that was not answered in time is NOT forgotten: if it is accepted later
  ("late acceptance") it is cancelled immediately and tracked like any other goal.
* Cancellation is only considered complete when (a) the cancel request is answered with
  ERROR_NONE / GOAL_TERMINATED (or the goal is already terminal) AND (b) the goal's terminal
  result arrives. Otherwise the goal stays *unresolved*.
* While any goal is unresolved, `motion_inhibited()` is True and `run()` refuses to send a new
  goal (returns INHIBITED). The mission then latches the safety gate's e-stop and aborts.
* Any exception / KeyboardInterrupt while waiting cancels the active goal before propagating.

All waiting happens in the caller's (mission) thread through the injected `wait` function,
never inside ROS callbacks, and every wait is bounded.
"""
from typing import Callable, Dict, List, Optional

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal

TERMINAL = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED'}
CANCEL_CODES = {CancelGoal.Response.ERROR_NONE: 'ERROR_NONE',
                CancelGoal.Response.ERROR_REJECTED: 'ERROR_REJECTED',
                CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID: 'ERROR_UNKNOWN_GOAL_ID',
                CancelGoal.Response.ERROR_GOAL_TERMINATED: 'ERROR_GOAL_TERMINATED'}


def _uuid(goal_id) -> bytes:
    return bytes(bytearray(int(x) for x in goal_id.uuid))


class _Tracked:
    def __init__(self, handle, result_future, label):
        self.handle = handle
        self.result_future = result_future
        self.label = label
        self.cancel_requested = False

    def terminal(self) -> bool:
        if not self.result_future.done():
            return False
        res = self.result_future.result()
        return res is not None and res.status in TERMINAL


class NavGoalRunner:
    def __init__(self, client, wait: Callable, now: Callable[[], float], logger=None,
                 accept_timeout_s: float = 15.0, cancel_timeout_s: float = 10.0,
                 terminal_timeout_s: float = 15.0):
        """wait(future, timeout_s, abort_check=None) -> bool (future done) ; now() -> seconds."""
        self._client = client
        self._wait = wait
        self._now = now
        self._log = logger
        self.accept_timeout_s = accept_timeout_s
        self.cancel_timeout_s = cancel_timeout_s
        self.terminal_timeout_s = terminal_timeout_s
        self._tracked: List[_Tracked] = []        # goals whose terminal result is not confirmed
        self._pending_sends: List = []            # (send_future, label) not answered in time
        self.events: List[Dict] = []              # audit trail for the mission record

    # ------------------------------------------------------------------ bookkeeping
    def _event(self, kind: str, label: str, detail: str = ''):
        self.events.append({'t': round(self._now(), 3), 'event': kind, 'goal': label,
                            'detail': detail})
        if self._log is not None:
            (self._log.warning if kind not in ('SENT', 'ACCEPTED', 'TERMINAL') else
             self._log.info)(f'[nav goal {label}] {kind} {detail}')

    def _poll(self):
        """Process late acceptances and drop goals that reached a terminal state."""
        still = []
        for fut, label in self._pending_sends:
            if not fut.done():
                still.append((fut, label))
                continue
            handle = fut.result()
            if handle is not None and handle.accepted:
                self._event('LATE_ACCEPTANCE', label, 'goal accepted after the client timeout '
                                                      '- cancelling it')
                tr = _Tracked(handle, handle.get_result_async(), label)
                self._tracked.append(tr)
                self._request_cancel(tr)
            else:
                self._event('LATE_REJECTION', label)
        self._pending_sends = still
        self._tracked = [t for t in self._tracked if not t.terminal()]

    def unresolved(self) -> List[str]:
        self._poll()
        return [t.label for t in self._tracked] + [lbl for _, lbl in self._pending_sends]

    def motion_inhibited(self) -> bool:
        return bool(self.unresolved())

    # ------------------------------------------------------------------ cancellation
    def _request_cancel(self, tr: _Tracked):
        tr.cancel_requested = True
        tr.cancel_future = tr.handle.cancel_goal_async()

    def _confirm_cancel(self, tr: _Tracked) -> (bool, str):
        if not tr.cancel_requested:
            self._request_cancel(tr)
        if not self._wait(tr.cancel_future, self.cancel_timeout_s):
            return False, 'no answer to the cancel request'
        resp = tr.cancel_future.result()
        code = getattr(resp, 'return_code', None)
        name = CANCEL_CODES.get(code, f'code {code}')
        ours = any(_uuid(g.goal_id) == _uuid(tr.handle.goal_id)
                   for g in getattr(resp, 'goals_canceling', []))
        if code == CancelGoal.Response.ERROR_NONE and not ours:
            # accepted cancel request that does not list our goal: only fine if already terminal
            if not tr.result_future.done():
                return False, 'cancel answered but our goal is not in goals_canceling'
        elif code not in (CancelGoal.Response.ERROR_NONE, CancelGoal.Response.ERROR_GOAL_TERMINATED):
            if not tr.result_future.done():
                return False, f'cancel refused ({name}) and goal still running'
        if not self._wait(tr.result_future, self.terminal_timeout_s):
            return False, f'cancel {name} but no terminal result within {self.terminal_timeout_s}s'
        res = tr.result_future.result()
        st = TERMINAL.get(getattr(res, 'status', None))
        if st is None:
            return False, f'non-terminal status {getattr(res, "status", None)} after cancel'
        return True, f'cancel {name}, terminal {st}'

    def cancel_all(self, reason: str) -> bool:
        """Cancel every tracked goal (and late acceptances) and confirm. True if all resolved."""
        ok = True
        for fut, label in list(self._pending_sends):
            self._wait(fut, self.accept_timeout_s)
        self._poll()
        for tr in list(self._tracked):
            done, detail = self._confirm_cancel(tr)
            self._event('CANCEL_CONFIRMED' if done else 'CANCEL_UNRESOLVED', tr.label,
                        f'{reason}: {detail}')
            ok &= done
        self._poll()
        return ok and not self.unresolved()

    # ------------------------------------------------------------------ main API
    def run(self, goal, label: str, timeout_s: float, abort_check: Optional[Callable] = None,
            feedback_cb: Optional[Callable] = None) -> Dict:
        t0 = self._now()
        out = lambda result, **kw: dict({'result': result,               # noqa: E731
                                         'duration_s': round(self._now() - t0, 2)}, **kw)
        if self.motion_inhibited():
            # give previous goals a bounded chance to resolve before refusing
            if not self.cancel_all('before new goal') or self.motion_inhibited():
                return out('INHIBITED', detail='previous goal(s) not confirmed terminated: '
                                               + ', '.join(self.unresolved()))
        send = self._client.send_goal_async(goal, feedback_callback=feedback_cb)
        self._event('SENT', label)
        if not self._wait(send, self.accept_timeout_s):
            self._pending_sends.append((send, label))
            self._event('NO_RESPONSE', label, f'no accept/reject within {self.accept_timeout_s}s; '
                                               'tracking for late acceptance')
            return out('NO_RESPONSE', detail='action server did not answer the goal request')
        handle = send.result()
        if handle is None or not handle.accepted:
            self._event('REJECTED', label)
            return out('REJECTED', detail='goal rejected by bt_navigator')
        tr = _Tracked(handle, handle.get_result_async(), label)
        self._tracked.append(tr)
        self._event('ACCEPTED', label)
        try:
            done = self._wait(tr.result_future, timeout_s, abort_check)
        except BaseException as e:   # MissionAborted, KeyboardInterrupt, ... -> never orphan
            ok, detail = self._confirm_cancel(tr)
            self._event('CANCEL_CONFIRMED' if ok else 'CANCEL_UNRESOLVED', label,
                        f'{type(e).__name__}: {detail}')
            self._poll()
            raise
        if not done:
            ok, detail = self._confirm_cancel(tr)
            self._event('CANCEL_CONFIRMED' if ok else 'CANCEL_UNRESOLVED', label,
                        f'timeout: {detail}')
            self._poll()
            return out('TIMEOUT', detail=f'no result within {timeout_s:.0f}s; {detail}',
                       cancel_confirmed=ok)
        res = tr.result_future.result()
        status = TERMINAL.get(getattr(res, 'status', None), f'STATUS_{getattr(res, "status", None)}')
        self._event('TERMINAL', label, status)
        self._poll()
        r = out(status)
        if status != 'SUCCEEDED':
            err = getattr(res.result, 'error_code', 0)
            msg = getattr(res.result, 'error_msg', '')
            r['detail'] = f'nav2 error_code={err} {msg}'.strip()
        return r
