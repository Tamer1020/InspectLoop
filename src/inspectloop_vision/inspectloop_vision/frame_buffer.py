"""Camera frame buffer and burst selection with freshness rules (Astra F5, ROS independent).

* Only frames stamped in [not_before, now + max_future] are eligible: frames taken before the
  observation request (e.g. at a previous viewpoint) are never used, and implausibly
  future-dated frames are rejected.
* ANY backward step of the node clock or of the (plausible) frame stamps is treated as a
  simulation/clock reset: the buffer is cleared and `time_epoch` increments. V3 (Astra V2 review
  F5): V2 ignored rollbacks smaller than 1 s, so a 0.8 s rollback kept frames from the old
  timeline eligible; now only a 1 ms tolerance for float rounding remains, and the node also
  forwards rclpy clock-jump callbacks through `notify_clock_jump()`.
* `invalidate()` drops every buffered frame (called when a new viewpoint epoch starts).
* A burst whose frames all have identical pixel digests is reported as frozen (a stuck camera
  re-publishing the same image with new stamps).
"""
from collections import deque
from dataclasses import dataclass
from typing import Any, Dict, List, Tuple


@dataclass
class BufferedFrame:
    stamp_ns: int
    digest: int
    payload: Any


class FrameBuffer:
    def __init__(self, maxlen: int = 30, reset_threshold_ns: int = 1_000_000,
                 max_future_ns: int = 200_000_000):
        self._frames = deque(maxlen=maxlen)
        self._reset_threshold_ns = reset_threshold_ns
        self._max_future_ns = max_future_ns
        self._last_good_stamp_ns = None
        self._last_now_ns = None
        self.time_epoch = 0

    def check_clock(self, now_ns: int) -> bool:
        """Call with the node clock; True (and buffer cleared) if the clock jumped backwards."""
        jumped = (self._last_now_ns is not None and
                  now_ns < self._last_now_ns - self._reset_threshold_ns)
        if jumped:
            self._reset()
            self._last_now_ns = now_ns
        else:
            self._last_now_ns = now_ns if self._last_now_ns is None else max(self._last_now_ns,
                                                                              now_ns)
        return jumped

    def _reset(self):
        self._frames.clear()
        self._last_good_stamp_ns = None
        self.time_epoch += 1

    def add(self, stamp_ns: int, digest: int, payload: Any, now_ns: int) -> bool:
        """Buffer a frame. Returns True if a clock reset was detected (buffer cleared first).

        Implausibly future-dated frames (stamp > now + max_future) are kept out of the
        reference used for reset detection, so a single bad stamp cannot wipe the buffer;
        select_burst() rejects them anyway."""
        reset = self.check_clock(now_ns)
        stamp_ns = int(stamp_ns)
        plausible = stamp_ns <= now_ns + self._max_future_ns
        if (plausible and self._last_good_stamp_ns is not None and
                stamp_ns < self._last_good_stamp_ns - self._reset_threshold_ns):
            self._reset()
            reset = True
        self._frames.append(BufferedFrame(stamp_ns, int(digest), payload))
        if plausible:
            self._last_good_stamp_ns = (stamp_ns if self._last_good_stamp_ns is None
                                        else max(self._last_good_stamp_ns, stamp_ns))
        return reset

    def notify_clock_jump(self):
        """Clock discontinuity reported by the ROS clock (jump callback)."""
        self._reset()
        self._last_now_ns = None

    def invalidate(self):
        self._frames.clear()

    def frames(self) -> List[BufferedFrame]:
        return list(self._frames)


def select_burst(frames: List[BufferedFrame], not_before_ns: int, now_ns: int,
                 max_future_ns: int, n: int) -> Tuple[List[BufferedFrame], Dict]:
    """Pick up to n distinct-stamp frames inside [not_before, now + max_future], oldest first."""
    stats = {'old': 0, 'future': 0, 'duplicate': 0, 'frozen': False}
    by_stamp = {}
    for f in frames:
        if f.stamp_ns < not_before_ns:
            stats['old'] += 1
        elif f.stamp_ns > now_ns + max_future_ns:
            stats['future'] += 1
        elif f.stamp_ns in by_stamp:
            stats['duplicate'] += 1
        else:
            by_stamp[f.stamp_ns] = f
    sel = [by_stamp[k] for k in sorted(by_stamp)][:n]
    stats['frozen'] = len(sel) >= 2 and len({f.digest for f in sel}) == 1
    return sel, stats
