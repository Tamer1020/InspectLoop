"""Frame reading in a dedicated worker process (V3).

Found in the V3 Gazebo demo: inside the ROS node the gauge reader competes for the Python GIL
with rclpy callbacks (/clock at ~1 kHz, /tf, camera images). Every OpenCV/NumPy call releases
and re-acquires the GIL, and each re-acquisition can wait for a whole switch interval, so a
5-frame burst that takes ~0.1 s in isolation took 8-12 s in the node. The new result-age limit
(F5) then rejected those results and the TF poses at the frame stamps had already left the
10 s TF buffer. Running read_frame in a separate process removes the contention.

This module deliberately does not import rclpy (it is loaded by a 'spawn' worker process).
"""
from typing import List, Optional

import numpy as np

from inspectloop_vision import gauge_reader as gr

_READER: Optional[gr.GaugeReader] = None


def read_frames(images: List[np.ndarray], tag_id: int,
                K: Optional[np.ndarray]) -> List[gr.FrameReading]:
    global _READER
    if _READER is None:
        _READER = gr.GaugeReader()
    return [_READER.read_frame(img, int(tag_id), K) for img in images]


def warm_up() -> bool:
    read_frames([np.zeros((48, 64, 3), np.uint8)], 0, None)
    return True
