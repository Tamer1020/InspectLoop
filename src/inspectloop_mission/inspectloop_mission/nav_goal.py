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
* Ownership starts BEFORE the request is issued (V3, Astra V2 review F4): the send future is
  registered as a pending send the moment `send_goal_async` returns, so an exception or
  KeyboardInterrupt anywhere in the send -> acceptance -> result lifecycle can never lose it.
  A goal accepted after such an interruption is cancelled and confirmed like any other.
* If `send_goal_async` itself raises, the request may or may not have reached the server:
  ownership is UNKNOWN, which permanently inhibits motion for this runner (fail safe; the
  mission then latches the gate e-stop).
* Cleanup never raises and never reports success while anything is unresolved: exceptions in
  cancel/result calls are recorded (CLEANUP_EXCEPTION) and leave the goal tracked.
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


class _Pending:
    """A goal request whose accept/reject answer has not been processed yet."""

    def __init__(self, label):
        self.label = label
        self.future = None


class _Tracked:
    def __init__(self, handle, label):
        self.handle = handle
        self.result_future = None
        self.cancel_future = None
        self.label = label
        self.cancel_requested = False
        self.reported = False          # terminal event already recorded

    def status_name(self) -> str:
        try:
            return TERMINAL.get(getattr(self.result_future.result(), 'status', None), '?')
        except Exception:  # noqa: BLE001
            return '?'

    def terminal(self) -> bool:
        if self.result_future is None or not self.result_future.done():
            return False
        try:
            res = self.result_future.result()
        except Exception:  # noqa: BLE001 - a failed result future proves nothing
            return False
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
        self._pending_sends: List[_Pending] = []  # requests without a processed answer
        self._ownership_unknown: List[str] = []   # send raised: the goal may exist unowned
        self.events: List[Dict] = []              # audit trail for the mission record

    # ------------------------------------------------------------------ bookkeeping
    def _event(self, kind: str, label: str, detail: str = ''):
        try:
            t = round(self._now(), 3)
        except Exception:  # noqa: BLE001 - bookkeeping must never raise
            t = None
        self.events.append({'t': t, 'event': kind, 'goal': label, 'detail': detail})
        if self._log is not None:
            msg = f'[nav goal {label}] {kind} {detail}'
            try:
                # rclpy forbids changing the severity of one call site: use two call sites
                if kind in ('SENT', 'ACCEPTED', 'TERMINAL'):
                    self._log.info(msg)
                else:
                    self._log.warning(msg)
            except Exception:  # noqa: BLE001
                pass

    def _track(self, handle, label) -> _Tracked:
        tr = _Tracked(handle, label)
        self._tracked.append(tr)               # owned before anything else can fail
        try:
            tr.result_future = handle.get_result_async()
        except Exception as e:  # noqa: BLE001 - stays tracked (unresolved) without a result
            self._event('CLEANUP_EXCEPTION', label, f'get_result_async: {e!r}')
        return tr

    def _poll(self):
        """Process late acceptances and drop goals that reached a terminal state."""
        still = []
        for pend in self._pending_sends:
            fut = pend.future
            if fut is None or not fut.done():
                still.append(pend)
                continue
            try:
                handle = fut.result()
            except Exception as e:  # noqa: BLE001 - the answer is unusable: ownership unknown
                self._mark_ownership_unknown(pend.label, f'send future failed: {e!r}')
                continue
            if handle is not None and getattr(handle, 'accepted', False):
                self._event('LATE_ACCEPTANCE', pend.label, 'goal accepted after the client '
                                                           'stopped waiting - cancelling it')
                tr = self._track(handle, pend.label)
                self._request_cancel(tr)
            else:
                self._event('LATE_REJECTION', pend.label)
        self._pending_sends = still
        keep = []
        for t in self._tracked:
            if not t.terminal():
                keep.append(t)
            elif not t.reported:
                t.reported = True
                self._event('TERMINAL', t.label, t.status_name() +
                            (' (after cancel request)' if t.cancel_requested else ''))
        self._tracked = keep

    def _mark_ownership_unknown(self, label: str, detail: str):
        self._ownership_unknown.append(label)
        self._event('OWNERSHIP_UNKNOWN', label, detail + ' - motion inhibited')

    def unresolved(self) -> List[str]:
        try:
            self._poll()
        except Exception as e:  # noqa: BLE001 - never hide state behind an exception
            self._event('CLEANUP_EXCEPTION', '*', f'poll: {e!r}')
        return ([t.label for t in self._tracked] + [p.label for p in self._pending_sends] +
                [f'{lbl} (ownership unknown)' for lbl in self._ownership_unknown])

    def motion_inhibited(self) -> bool:
        return bool(self.unresolved())

    # ------------------------------------------------------------------ cancellation
    def _request_cancel(self, tr: _Tracked):
        tr.cancel_requested = True
        try:
            tr.cancel_future = tr.handle.cancel_goal_async()
        except Exception as e:  # noqa: BLE001
            tr.cancel_future = None
            self._event('CLEANUP_EXCEPTION', tr.label, f'cancel_goal_async: {e!r}')

    def _confirm_cancel(self, tr: _Tracked) -> (bool, str):
        """Never raises (BaseException included): cleanup must always reach a verdict."""
        try:
            return self._confirm_cancel_inner(tr)
        except BaseException as e:  # noqa: BLE001 - incl. a 2nd KeyboardInterrupt in cleanup
            self._event('CLEANUP_EXCEPTION', tr.label, f'{type(e).__name__}: {e}')
            return False, f'cleanup raised {type(e).__name__}: {e}'

    def _confirm_cancel_inner(self, tr: _Tracked) -> (bool, str):
        if tr.terminal():
            return True, f'already terminal ({tr.status_name()})'
        if not tr.cancel_requested or tr.cancel_future is None:
            self._request_cancel(tr)
        if tr.cancel_future is None:
            return False, 'cancel request could not be sent'
        if not self._wait(tr.cancel_future, self.cancel_timeout_s):
            return False, 'no answer to the cancel request'
        resp = tr.cancel_future.result()
        code = getattr(resp, 'return_code', None)
        name = CANCEL_CODES.get(code, f'code {code}')
        ours = any(_uuid(g.goal_id) == _uuid(tr.handle.goal_id)
                   for g in getattr(resp, 'goals_canceling', []))
        done_already = tr.result_future is not None and tr.result_future.done()
        if code == CancelGoal.Response.ERROR_NONE and not ours:
            # accepted cancel request that does not list our goal: only fine if already terminal
            if not done_already:
                return False, 'cancel answered but our goal is not in goals_canceling'
        elif code not in (CancelGoal.Response.ERROR_NONE, CancelGoal.Response.ERROR_GOAL_TERMINATED):
            if not done_already:
                return False, f'cancel refused ({name}) and goal still running'
        if tr.result_future is None:
            return False, f'cancel {name} but no result future to confirm termination'
        if not self._wait(tr.result_future, self.terminal_timeout_s):
            return False, f'cancel {name} but no terminal result within {self.terminal_timeout_s}s'
        res = tr.result_future.result()
        st = TERMINAL.get(getattr(res, 'status', None))
        if st is None:
            return False, f'non-terminal status {getattr(res, "status", None)} after cancel'
        return True, f'cancel {name}, terminal {st}'

    def cancel_all(self, reason: str) -> bool:
        """Cancel every tracked goal (and late acceptances) and confirm.

        True only if nothing is unresolved afterwards. Never raises: an exception inside the
        cleanup is recorded and makes the result False (the goal stays tracked/inhibiting)."""
        ok = True
        try:
            for pend in list(self._pending_sends):
                if pend.future is not None and not pend.future.done():
                    try:
                        self._wait(pend.future, self.accept_timeout_s)
                    except BaseException as e:  # noqa: BLE001
                        ok = False
                        self._event('CLEANUP_EXCEPTION', pend.label,
                                    f'waiting for acceptance: {type(e).__name__}: {e}')
            self._poll()
            for tr in list(self._tracked):
                done, detail = self._confirm_cancel(tr)
                self._event('CANCEL_CONFIRMED' if done else 'CANCEL_UNRESOLVED', tr.label,
                            f'{reason}: {detail}')
                ok &= done
        except BaseException as e:  # noqa: BLE001
            ok = False
            self._event('CLEANUP_EXCEPTION', '*', f'{reason}: {type(e).__name__}: {e}')
        left = self.unresolved()
        if left:
            self._event('CLEANUP_INCOMPLETE', ','.join(left), reason)
        return ok and not left

    def _cleanup_after(self, label: str, why: str):
        """Called with an exception in flight: resolve everything we own, never raise."""
        ok = self.cancel_all(why)
        if not ok:
            self._event('CANCEL_UNRESOLVED', label, f'{why}: motion inhibited')

    # ------------------------------------------------------------------ main API
    def run(self, goal, label: str, timeout_s: float, abort_check: Optional[Callable] = None,
            feedback_cb: Optional[Callable] = None) -> Dict:
        t0 = self._now()
        out = lambda result, **kw: dict({'result': result,               # noqa: E731
                                         'duration_s': round(self._now() - t0, 2)}, **kw)
        if self._ownership_unknown:
            return out('INHIBITED', detail='ownership of earlier goal request(s) unknown: '
                                           + ', '.join(self._ownership_unknown))
        if self.motion_inhibited():
            # give previous goals a bounded chance to resolve before refusing
            if not self.cancel_all('before new goal') or self.motion_inhibited():
                return out('INHIBITED', detail='previous goal(s) not confirmed terminated: '
                                               + ', '.join(self.unresolved()))
        pend = _Pending(label)
        try:
            pend.future = self._client.send_goal_async(goal, feedback_callback=feedback_cb)
        except BaseException as e:
            self._mark_ownership_unknown(label, f'send_goal_async raised {type(e).__name__}: {e}')
            raise
        self._pending_sends.append(pend)          # owned from here on, whatever happens next
        try:
            return self._run_owned(pend, label, timeout_s, abort_check, out)
        except BaseException as e:   # MissionAborted, KeyboardInterrupt, ... -> never orphan
            self._cleanup_after(label, f'{type(e).__name__}: {e}')
            raise

    def _run_owned(self, pend: _Pending, label, timeout_s, abort_check, out) -> Dict:
        self._event('SENT', label)
        if not self._wait(pend.future, self.accept_timeout_s):
            self._event('NO_RESPONSE', label, f'no accept/reject within {self.accept_timeout_s}s; '
                                               'tracking for late acceptance')
            return out('NO_RESPONSE', detail='action server did not answer the goal request')
        handle = pend.future.result()
        if handle is None or not handle.accepted:
            self._pending_sends.remove(pend)
            self._event('REJECTED', label)
            return out('REJECTED', detail='goal rejected by bt_navigator')
        tr = self._track(handle, label)
        self._pending_sends.remove(pend)          # ownership moved to the tracked goal
        self._event('ACCEPTED', label)
        if tr.result_future is None:
            ok, detail = self._confirm_cancel(tr)
            return out('ABORTED', detail=f'no result future; cancel: {detail}', cancel_confirmed=ok)
        done = self._wait(tr.result_future, timeout_s, abort_check)
        if not done:
            ok, detail = self._confirm_cancel(tr)
            self._event('CANCEL_CONFIRMED' if ok else 'CANCEL_UNRESOLVED', label,
                        f'timeout: {detail}')
            self._poll()
            return out('TIMEOUT', detail=f'no result within {timeout_s:.0f}s; {detail}',
                       cancel_confirmed=ok)
        res = tr.result_future.result()
        status = TERMINAL.get(getattr(res, 'status', None), f'STATUS_{getattr(res, "status", None)}')
        tr.reported = True
        self._event('TERMINAL', label, status)
        self._poll()
        r = out(status)
        if status != 'SUCCEEDED':
            err = getattr(res.result, 'error_code', 0)
            msg = getattr(res.result, 'error_msg', '')
            r['detail'] = f'nav2 error_code={err} {msg}'.strip()
        return r
