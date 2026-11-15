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
* Ownership starts BEFORE the request is issued (V3.1, audit F4): the pending record is
  appended to `_pending_sends` before `send_goal_async` is called, and the send itself runs
  inside the exception-cleanup scope. There is therefore no gap between "request sent" and
  "request owned" in which an exception or KeyboardInterrupt (SIGINT) can orphan a goal. A goal
  accepted after such an interruption is cancelled and confirmed like any other.
* If `send_goal_async` raises, or an interruption lands after it returned but before its
  future was stored, the request may have reached the server while we hold no handle:
  ownership is UNKNOWN, which permanently inhibits motion for this runner and makes every
  cleanup report failure (fail safe; the mission then latches the gate e-stop).
* A pending record with no future yet (send in progress) counts as unresolved.
* Goal IDs are chosen by the runner BEFORE the request is sent (V3.2, F4 hardening) and passed
  as `goal_uuid`, so even an UNKNOWN-ownership goal has a known ID. With a `goal_id_fallback`
  (see `GoalIdCanceller`) cleanup cancels exactly that ID through the action's CancelGoal
  service - never the cancel-all form (zero ID): other clients (RViz Nav2 panel, waypoint
  follower, operator CLI) share `navigate_to_pose` and their goals must not be cancelled.
  An unknown-ownership goal is resolved ONLY when the server's own status topic reports that
  ID terminal (after ERROR_NONE listing it, or ERROR_GOAL_TERMINATED), and - when a
  `motion_stopped` check is configured - the robot is then observed stationary. A sent cancel
  request, ERROR_UNKNOWN_GOAL_ID (request lost or still in flight), ERROR_REJECTED, no answer,
  or an unobservable status all leave it unresolved (motion stays inhibited, fail closed).
  If the status topic later shows the ID active (late arrival), a cancel is requested at once.
* Cleanup never raises and never reports success while anything is unresolved: exceptions in
  cancel/result calls are recorded (CLEANUP_EXCEPTION) and leave the goal tracked.
* Any exception / KeyboardInterrupt while waiting cancels the active goal before propagating.

All waiting happens in the caller's (mission) thread through the injected `wait` function,
never inside ROS callbacks, and every wait is bounded.
"""
import threading
from typing import Callable, Dict, List, Optional
import uuid as uuidlib

from action_msgs.msg import GoalStatus, GoalStatusArray
from action_msgs.srv import CancelGoal
from unique_identifier_msgs.msg import UUID

TERMINAL = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
            GoalStatus.STATUS_CANCELED: 'CANCELED'}
CANCEL_CODES = {CancelGoal.Response.ERROR_NONE: 'ERROR_NONE',
                CancelGoal.Response.ERROR_REJECTED: 'ERROR_REJECTED',
                CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID: 'ERROR_UNKNOWN_GOAL_ID',
                CancelGoal.Response.ERROR_GOAL_TERMINATED: 'ERROR_GOAL_TERMINATED'}


def _uuid(goal_id) -> bytes:
    return bytes(bytearray(int(x) for x in goal_id.uuid))


def random_goal_id() -> UUID:
    return UUID(uuid=list(uuidlib.uuid4().bytes))


class _Pending:
    """A goal request whose accept/reject answer has not been processed yet."""

    def __init__(self, label):
        self.label = label
        self.future = None
        self.goal_id = None            # chosen by us before sending
        self.send_called = False       # send_goal_async may have transmitted the request


class _Unknown:
    """A goal request that may exist on the server although we hold no handle for it."""

    def __init__(self, label, goal_id):
        self.label = label
        self.goal_id = goal_id         # None only if the request was never sent
        self.cancel_future = None
        self.cancel_requests = 0


class _Condition:
    """Future-like `done()` for the injected wait(): polls a predicate, never raises."""

    def __init__(self, predicate):
        self._predicate = predicate

    def done(self) -> bool:
        try:
            return bool(self._predicate())
        except Exception:  # noqa: BLE001
            return False


class GoalIdCanceller:
    """Targeted cancellation + server-side status for goal IDs we chose (ROS adapter).

    * cancel(goal_id) sends CancelGoal for exactly that ID (stamp 0) on
      `<action>/_action/cancel_goal` and returns the response future.
    * status(goal_id) is the status the action server last published for that ID on
      `<action>/_action/status`, or None if the server does not list it (unknown / never
      received / result expired) or no status message was received yet.
    """

    def __init__(self, node, action_name: str, callback_group=None):
        from rclpy.qos import qos_profile_action_status_default, qos_profile_services_default
        self._lock = threading.Lock()
        self._latest: Optional[Dict[bytes, int]] = None
        self._client = node.create_client(CancelGoal, f'{action_name}/_action/cancel_goal',
                                          qos_profile=qos_profile_services_default,
                                          callback_group=callback_group)
        self._sub = node.create_subscription(GoalStatusArray, f'{action_name}/_action/status',
                                             self._on_status, qos_profile_action_status_default,
                                             callback_group=callback_group)

    def _on_status(self, msg):
        latest = {_uuid(st.goal_info.goal_id): int(st.status) for st in msg.status_list}
        with self._lock:
            self._latest = latest

    def status(self, goal_id) -> Optional[int]:
        with self._lock:
            return None if self._latest is None else self._latest.get(_uuid(goal_id))

    def cancel(self, goal_id):
        if not any(int(x) for x in goal_id.uuid):
            raise ValueError('refusing the cancel-all form (zero goal id)')
        req = CancelGoal.Request()
        req.goal_info.goal_id = goal_id          # stamp stays 0: this goal ID only
        return self._client.call_async(req)


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
                 terminal_timeout_s: float = 15.0, goal_id_fallback=None,
                 motion_stopped: Optional[Callable[[], bool]] = None,
                 new_goal_id: Callable[[], UUID] = random_goal_id):
        """wait(future, timeout_s, abort_check=None) -> bool (future done) ; now() -> seconds.

        goal_id_fallback: object with cancel(goal_id) -> future and status(goal_id) ->
        Optional[int] (see GoalIdCanceller); without it unknown ownership is never resolved.
        motion_stopped: () -> bool, required True after a fallback resolution."""
        self._fallback = goal_id_fallback
        self._motion_stopped = motion_stopped
        self._new_goal_id = new_goal_id
        self._client = client
        self._wait = wait
        self._now = now
        self._log = logger
        self.accept_timeout_s = accept_timeout_s
        self.cancel_timeout_s = cancel_timeout_s
        self.terminal_timeout_s = terminal_timeout_s
        self._tracked: List[_Tracked] = []        # goals whose terminal result is not confirmed
        self._pending_sends: List[_Pending] = []  # requests without a processed answer
        self._ownership_unknown: List[_Unknown] = []  # the goal may exist unowned
        self.events: List[Dict] = []              # audit trail for the mission record
        self._resolved_by_fallback = False        # motion check pending after a fallback

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
                self._mark_ownership_unknown(pend.label, f'send future failed: {e!r}',
                                             pend.goal_id)
                continue
            if any(t.handle is handle for t in self._tracked):
                continue                   # interrupted between _track() and pending release
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
        self._poll_unknown()

    def _server_status(self, u: _Unknown) -> Optional[int]:
        if self._fallback is None or u.goal_id is None:
            return None
        try:
            return self._fallback.status(u.goal_id)
        except Exception:  # noqa: BLE001 - unobservable
            return None

    def _poll_unknown(self):
        """Server status of unknown-ownership goals: terminal -> resolved; active -> cancel."""
        still = []
        for u in self._ownership_unknown:
            st = self._server_status(u)
            if st in TERMINAL:
                self._event('UNKNOWN_RESOLVED', u.label,
                            f'server status {TERMINAL[st]} for our goal id')
                self._resolved_by_fallback = True
                continue
            if st is not None and u.cancel_future is None:
                self._event('UNKNOWN_GOAL_ACTIVE', u.label,
                            f'server lists our goal id with status {st} - cancelling it')
                self._request_cancel_by_id(u)
            still.append(u)
        self._ownership_unknown = still

    def _request_cancel_by_id(self, u: _Unknown):
        if self._fallback is None or u.goal_id is None:
            return
        try:
            u.cancel_future = self._fallback.cancel(u.goal_id)
            u.cancel_requests += 1
        except Exception as e:  # noqa: BLE001
            u.cancel_future = None
            self._event('CLEANUP_EXCEPTION', u.label, f'cancel by goal id: {e!r}')

    def _mark_ownership_unknown(self, label: str, detail: str, goal_id=None):
        self._ownership_unknown.append(_Unknown(label, goal_id))
        self._event('OWNERSHIP_UNKNOWN', label, detail + ' - motion inhibited')

    def _resolve_unknown(self, u: _Unknown) -> (bool, str):
        """Never raises (BaseException included)."""
        try:
            return self._resolve_unknown_inner(u)
        except BaseException as e:  # noqa: BLE001
            self._event('CLEANUP_EXCEPTION', u.label, f'{type(e).__name__}: {e}')
            return False, f'fallback raised {type(e).__name__}: {e}'

    def _resolve_unknown_inner(self, u: _Unknown) -> (bool, str):
        if self._fallback is None:
            return False, 'no goal-id fallback configured'
        if u.goal_id is None:
            return False, 'goal id unknown'
        st = self._server_status(u)
        if st in TERMINAL:
            return True, f'server status {TERMINAL[st]}'
        if u.cancel_future is None or (u.cancel_future.done() and st is not None):
            self._request_cancel_by_id(u)       # (re)request: first time or goal now listed
        if u.cancel_future is None:
            return False, 'cancel-by-id request could not be sent'
        if not self._wait(u.cancel_future, self.cancel_timeout_s):
            u.cancel_future = None              # retry next time
            return False, 'no answer to the cancel-by-id request'
        resp = u.cancel_future.result()
        code = getattr(resp, 'return_code', None)
        name = CANCEL_CODES.get(code, f'code {code}')
        listed = any(_uuid(g.goal_id) == _uuid(u.goal_id)
                     for g in getattr(resp, 'goals_canceling', []))
        if code == CancelGoal.Response.ERROR_NONE and not listed:
            return False, 'cancel answered ERROR_NONE without our goal id'
        if code == CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID:
            u.cancel_future = None
            return False, ('server does not know our goal id (request lost or still in '
                           'flight) - stays unresolved')
        if code not in (CancelGoal.Response.ERROR_NONE,
                        CancelGoal.Response.ERROR_GOAL_TERMINATED):
            return False, f'cancel-by-id refused ({name})'
        terminal = _Condition(lambda: self._server_status(u) in TERMINAL)
        if not self._wait(terminal, self.terminal_timeout_s):
            return False, (f'cancel-by-id {name} but the server status never showed our goal '
                           f'terminal within {self.terminal_timeout_s}s')
        return True, f'cancel-by-id {name}, server status {TERMINAL[self._server_status(u)]}'

    def unresolved(self) -> List[str]:
        try:
            self._poll()
        except Exception as e:  # noqa: BLE001 - never hide state behind an exception
            self._event('CLEANUP_EXCEPTION', '*', f'poll: {e!r}')
        return ([t.label for t in self._tracked] + [p.label for p in self._pending_sends] +
                [f'{u.label} (ownership unknown)' for u in self._ownership_unknown])

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
        if tr.result_future is None:
            # get_result_async failed or was interrupted inside _track(): ask again, otherwise
            # termination could never be confirmed and the goal would stay stranded
            try:
                tr.result_future = tr.handle.get_result_async()
            except Exception as e:  # noqa: BLE001
                self._event('CLEANUP_EXCEPTION', tr.label, f'get_result_async retry: {e!r}')
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
            self._poll_unknown()
            for u in list(self._ownership_unknown):
                done, detail = self._resolve_unknown(u)
                self._event('UNKNOWN_RESOLVED' if done else 'UNKNOWN_UNRESOLVED', u.label,
                            f'{reason}: {detail}')
                if done:
                    self._ownership_unknown.remove(u)
                    self._resolved_by_fallback = True
                ok &= done
            if self._resolved_by_fallback and self._motion_stopped is not None:
                # a goal we could not see may have moved the robot: require it to be stationary
                if not self._wait(_Condition(self._motion_stopped), self.terminal_timeout_s):
                    ok = False
                    self._event('MOTION_NOT_CONFIRMED_STOPPED', '*',
                                f'{reason}: robot not observed stationary after the '
                                'unknown-ownership goal was resolved')
                else:
                    self._resolved_by_fallback = False
        except BaseException as e:  # noqa: BLE001
            ok = False
            self._event('CLEANUP_EXCEPTION', '*', f'{reason}: {type(e).__name__}: {e}')
        try:
            left = self.unresolved()
        except BaseException as e:  # noqa: BLE001 - e.g. a 2nd Ctrl-C: never raise, never clean
            self._event('CLEANUP_EXCEPTION', '*', f'{reason}: {type(e).__name__}: {e}')
            return False
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
        if self._ownership_unknown or self.motion_inhibited():
            # give previous goals a bounded chance to resolve before refusing
            if not self.cancel_all('before new goal') or self.motion_inhibited():
                return out('INHIBITED', detail='previous goal(s) not confirmed terminated: '
                                               + ', '.join(self.unresolved()))
        # Ownership is established BEFORE the request can exist (V3.1, F4): the pending record
        # is registered first, so there is no instruction between "request sent" and "request
        # owned" where an exception / KeyboardInterrupt could orphan it.
        pend = _Pending(label)
        try:
            try:
                pend.goal_id = self._new_goal_id()
                self._pending_sends.append(pend)
                pend.send_called = True
                pend.future = self._client.send_goal_async(goal, feedback_callback=feedback_cb,
                                                           goal_uuid=pend.goal_id)
            except BaseException as e:
                # Raised inside send_goal_async, or after it returned but before its future
                # was stored: the request may have reached the server and we hold no handle
                # to it -> ownership UNKNOWN (motion inhibited; resolvable only through the
                # goal-id fallback), never "nothing sent".
                if pend.future is None:
                    self._remove_pending(pend)
                    if pend.send_called:
                        self._mark_ownership_unknown(
                            label, f'interrupted while sending ({type(e).__name__}: {e})',
                            pend.goal_id)
                    else:
                        self._event('NOT_SENT', label, f'{type(e).__name__} before sending')
                raise
            return self._run_owned(pend, label, timeout_s, abort_check, out)
        except BaseException as e:   # MissionAborted, KeyboardInterrupt, ... -> never orphan
            self._cleanup_after(label, f'{type(e).__name__}: {e}')
            raise

    def _remove_pending(self, pend: _Pending):
        try:
            self._pending_sends.remove(pend)
        except ValueError:
            pass

    def _run_owned(self, pend: _Pending, label, timeout_s, abort_check, out) -> Dict:
        self._event('SENT', label)
        if not self._wait(pend.future, self.accept_timeout_s):
            self._event('NO_RESPONSE', label, f'no accept/reject within {self.accept_timeout_s}s; '
                                               'tracking for late acceptance')
            return out('NO_RESPONSE', detail='action server did not answer the goal request')
        handle = pend.future.result()
        if handle is None or not handle.accepted:
            self._remove_pending(pend)
            self._event('REJECTED', label)
            return out('REJECTED', detail='goal rejected by bt_navigator')
        tr = self._track(handle, label)            # tracked first ...
        self._remove_pending(pend)                 # ... then the pending record is released
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
