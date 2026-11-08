"""V3: frame reading in a spawn worker process gives the same result as in-process reading."""
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
import os

import cv2

from inspectloop_vision import gauge_reader as gr
from inspectloop_vision import reader_worker

DATA = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'data')


def test_worker_process_reads_a_gazebo_frame_like_the_in_process_reader():
    img = cv2.imread(os.path.join(DATA, 'gz_pg14_frontal_truth8.7.png'))
    local = gr.GaugeReader().read_frame(img, 14)
    with ProcessPoolExecutor(1, mp_context=multiprocessing.get_context('spawn')) as pool:
        assert pool.submit(reader_worker.warm_up).result(timeout=60)
        remote = pool.submit(reader_worker.read_frames, [img, img], 14, None).result(timeout=60)
    assert len(remote) == 2
    assert abs(remote[0].value - local.value) < 1e-9
    assert abs(remote[0].value - 8.7) < 0.05
    obs = gr.fuse(remote, 2)
    assert obs.has_value
