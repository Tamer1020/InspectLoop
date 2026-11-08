"""Mission-side re-validation of a gauge observation before it is used (Astra F5).

The vision node already enforces freshness, but the mission does not trust a result merely
because it arrived: a late reply to an abandoned request, a reply from a previous viewpoint,
a capture during a clock reset or while the robot was not at (or not stopped at) the
viewpoint must never become evidence for the current viewpoint.

V3 (Astra V2 review F5) adds, at the moment the mission CONSUMES a result:
* an explicit result-age limit (now - last contributing frame <= max_result_age_s);
* chronological ordering request <= first frame <= last frame <= now (+ future tolerance);
* the mission's clock epoch: any backward step of the ROS clock between request and
  consumption (detected by a jump callback, not by a size threshold) invalidates the result;
* mission identity (observation ids are '<mission_id>/<gauge>/<viewpoint>/<n>');
* a value must come with frames and with a known acquisition-time pose;
* odometry used for the settle check must itself be fresh (stamp-based, not receive-based).
"""
from dataclasses import dataclass
import math
from typing import Dict, List, Optional, Sequence, Tuple


@dataclass
class ExpectedObservation:
    observation_id: str
    viewpoint_epoch: int
    request_time_s: float          # mission ROS time when the request was sent
    now_s: float                   # mission ROS time when the reply is consumed
    x: float                       # viewpoint goal (map)
    y: float
    yaw: float
    max_pose_error_m: float = 0.35
    max_pose_error_rad: float = 0.40
    max_motion_m: float = 0.02
    max_motion_rad: float = 0.02
    max_future_s: float = 0.2
    stamp_slack_s: float = 0.05
    max_result_age_s: float = 8.0     # sim seconds from the last contributing frame to consumption
    clock_epoch_request: int = 0      # mission clock epoch when the request was sent
    clock_epoch_now: int = 0          # ... and when the reply is consumed
    mission_id: str = ''              # '' = do not check


def _ang(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def validate_observation(r: Dict, e: ExpectedObservation) -> List[str]:
    """Return a list of problems (empty = the observation may be used)."""
    p = []
    if r.get('observation_id') != e.observation_id:
        p.append(f"reply for observation {r.get('observation_id')!r}, expected {e.observation_id!r}")
    if r.get('viewpoint_epoch') != e.viewpoint_epoch:
        p.append(f"viewpoint epoch {r.get('viewpoint_epoch')} != current {e.viewpoint_epoch}")
    oid = r.get('observation_id')
    if e.mission_id and not (isinstance(oid, str) and oid.split('/')[0] == e.mission_id):
        p.append(f'reply {oid!r} does not belong to mission {e.mission_id!r}')
    if e.clock_epoch_now != e.clock_epoch_request:
        p.append(f'clock discontinuity between request and consumption (clock epoch '
                 f'{e.clock_epoch_request} -> {e.clock_epoch_now})')
    if e.now_s < e.request_time_s - e.stamp_slack_s:
        p.append(f'clock went backwards during the observation ({e.request_time_s:.2f} -> '
                 f'{e.now_s:.2f} s)')
    if int(r.get('frames_received') or 0) == 0:
        if r.get('has_value') is True:
            p.append('value reported without any contributing frame')
        return p                    # nothing captured: the reader reports the fault itself
    first, last = r.get('first_frame_stamp_s'), r.get('last_frame_stamp_s')
    if not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v)
               for v in (first, last)):
        p.append('frame stamps missing')
        return p
    if first > last:
        p.append(f'frame stamps out of order (first {first:.3f} s > last {last:.3f} s)')
    if first < e.request_time_s - e.stamp_slack_s:
        p.append(f'frame stamped {first:.2f} s precedes the request ({e.request_time_s:.2f} s)')
    if last > e.now_s + e.max_future_s:
        p.append(f'frame stamped {last:.2f} s is in the future (now {e.now_s:.2f} s)')
    age = e.now_s - last
    if age > e.max_result_age_s:
        p.append(f'result too old when consumed: last frame {age:.2f} s old '
                 f'(limit {e.max_result_age_s:.1f} s)')
    if not r.get('pose_known'):
        p.append('robot pose at acquisition time unknown')
    else:
        d = math.hypot(r['acq_x'] - e.x, r['acq_y'] - e.y)
        dyaw = abs(_ang(r['acq_yaw'] - e.yaw))
        if d > e.max_pose_error_m:
            p.append(f'captured {d:.2f} m away from the viewpoint')
        if dyaw > e.max_pose_error_rad:
            p.append(f'captured with heading error {dyaw:.2f} rad')
        if (r.get('pose_motion_m', 0.0) > e.max_motion_m or
                r.get('pose_motion_rad', 0.0) > e.max_motion_rad):
            p.append(f"robot moved during the burst ({r.get('pose_motion_m', 0):.3f} m, "
                     f"{r.get('pose_motion_rad', 0):.3f} rad)")
    return p


def is_settled(samples: Sequence[Tuple[float, float, float]], window_s: float, max_v: float,
               max_w: float, now_s: Optional[float] = None, max_age_s: float = 0.3,
               min_samples: int = 3, max_future_s: float = 0.1) -> bool:
    """samples: (header stamp, |v|, |w|) from odometry, oldest first. True if the robot has been
    at rest for at least window_s up to now.

    V3 (F5): with now_s given, the newest sample must be at most max_age_s old and not in the
    future, and the window must contain at least min_samples samples - a stalled or sparse
    odometry stream cannot prove that the robot stands still."""
    if not samples:
        return False
    t_end = samples[-1][0]
    if now_s is not None:
        if t_end > now_s + max_future_s or now_s - t_end > max_age_s:
            return False
    recent = [s for s in samples if s[0] >= t_end - window_s]
    if t_end - samples[0][0] < window_s or len(recent) < min_samples:
        return False
    return all(abs(v) <= max_v and abs(w) <= max_w for _, v, w in recent)
