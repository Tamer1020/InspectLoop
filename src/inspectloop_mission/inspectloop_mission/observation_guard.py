"""Mission-side re-validation of a gauge observation before it is used (Astra F5).

The vision node already enforces freshness, but the mission does not trust a result merely
because it arrived: a late reply to an abandoned request, a reply from a previous viewpoint,
a capture during a clock reset or while the robot was not at (or not stopped at) the
viewpoint must never become evidence for the current viewpoint.
"""
from dataclasses import dataclass
import math
from typing import Dict, List, Sequence, Tuple


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


def _ang(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def validate_observation(r: Dict, e: ExpectedObservation) -> List[str]:
    """Return a list of problems (empty = the observation may be used)."""
    p = []
    if r.get('observation_id') != e.observation_id:
        p.append(f"reply for observation {r.get('observation_id')!r}, expected {e.observation_id!r}")
    if r.get('viewpoint_epoch') != e.viewpoint_epoch:
        p.append(f"viewpoint epoch {r.get('viewpoint_epoch')} != current {e.viewpoint_epoch}")
    if e.now_s < e.request_time_s - e.stamp_slack_s:
        p.append(f'clock went backwards during the observation ({e.request_time_s:.2f} -> '
                 f'{e.now_s:.2f} s)')
    if int(r.get('frames_received') or 0) == 0:
        return p                    # nothing captured: the reader reports NO_IMAGE itself
    first, last = r.get('first_frame_stamp_s'), r.get('last_frame_stamp_s')
    if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (first, last)):
        p.append('frame stamps missing')
        return p
    if first < e.request_time_s - e.stamp_slack_s:
        p.append(f'frame stamped {first:.2f} s precedes the request ({e.request_time_s:.2f} s)')
    if last > e.now_s + e.max_future_s:
        p.append(f'frame stamped {last:.2f} s is in the future (now {e.now_s:.2f} s)')
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
               max_w: float) -> bool:
    """samples: (t, |v|, |w|) from odometry, oldest first. True if the robot has been at rest
    for at least window_s up to the newest sample."""
    if not samples:
        return False
    t_end = samples[-1][0]
    recent = [s for s in samples if s[0] >= t_end - window_s]
    if t_end - samples[0][0] < window_s or not recent:
        return False
    return all(abs(v) <= max_v and abs(w) <= max_w for _, v, w in recent)
