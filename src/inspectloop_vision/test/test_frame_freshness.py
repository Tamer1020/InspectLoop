"""Astra F5 (vision side): frame selection must reject stale, future-dated, frozen and
pre-reset frames, and never reuse a frame from a previous viewpoint epoch."""
from inspectloop_vision.frame_buffer import FrameBuffer, select_burst

S = 1_000_000_000


def test_future_dated_frames_are_rejected():
    buf = FrameBuffer()
    for t, d in ((10.0, 1), (10.1, 2), (60.0, 3), (10.2, 4)):   # 60 s: implausible future stamp
        buf.add(int(t * S), d, payload=t, now_ns=int((t if t < 50 else 10.15) * S))
    sel, stats = select_burst(buf.frames(), not_before_ns=int(9.9 * S), now_ns=int(10.3 * S),
                              max_future_ns=int(0.2 * S), n=5)
    assert [f.payload for f in sel] == [10.0, 10.1, 10.2]
    assert stats['future'] == 1


def test_frames_before_request_are_never_used():
    buf = FrameBuffer()
    for t in (5.0, 5.1, 5.2, 6.0, 6.1):
        buf.add(int(t * S), int(t * 10), payload=t, now_ns=int(t * S))
    sel, stats = select_burst(buf.frames(), int(5.5 * S), int(6.2 * S), int(0.2 * S), 5)
    assert [f.payload for f in sel] == [6.0, 6.1]
    assert stats['old'] == 3


def test_duplicate_stamps_count_once():
    buf = FrameBuffer()
    buf.add(int(7.0 * S), 1, payload='a', now_ns=int(7.0 * S))
    buf.add(int(7.0 * S), 1, payload='a2', now_ns=int(7.0 * S))
    sel, _ = select_burst(buf.frames(), int(6.0 * S), int(7.1 * S), int(0.2 * S), 5)
    assert len(sel) == 1


def test_clock_reset_clears_buffer_and_bumps_epoch():
    buf = FrameBuffer(reset_threshold_ns=int(1.0 * S))
    for t in (100.0, 100.1, 100.2):
        buf.add(int(t * S), 1, payload=t, now_ns=int(t * S))
    e0 = buf.time_epoch
    buf.add(int(2.0 * S), 2, payload=2.0, now_ns=int(2.0 * S))   # simulation restarted
    assert buf.time_epoch == e0 + 1
    assert [f.payload for f in buf.frames()] == [2.0]


def test_clock_reset_detected_from_node_clock_even_without_new_frames():
    buf = FrameBuffer(reset_threshold_ns=int(1.0 * S))
    buf.add(int(100.0 * S), 1, payload=1, now_ns=int(100.0 * S))
    assert buf.check_clock(int(3.0 * S)) is True
    assert buf.frames() == []


def test_single_future_frame_cannot_wipe_the_buffer():
    buf = FrameBuffer(reset_threshold_ns=int(1.0 * S))
    buf.add(int(10.0 * S), 1, payload=10.0, now_ns=int(10.0 * S))
    buf.add(int(500.0 * S), 2, payload=500.0, now_ns=int(10.05 * S))
    assert buf.add(int(10.1 * S), 3, payload=10.1, now_ns=int(10.1 * S)) is False
    assert len(buf.frames()) == 3


def test_invalidate_drops_everything_for_a_new_viewpoint():
    buf = FrameBuffer()
    for t in (1.0, 1.1):
        buf.add(int(t * S), 1, payload=t, now_ns=int(t * S))
    buf.invalidate()
    assert buf.frames() == []


def test_frozen_frames_detected():
    buf = FrameBuffer()
    for i in range(5):
        buf.add(int((3.0 + 0.1 * i) * S), 1234, payload=i,      # new stamps, identical pixels
                now_ns=int((3.0 + 0.1 * i) * S))
    sel, stats = select_burst(buf.frames(), int(2.9 * S), int(3.5 * S), int(0.2 * S), 5)
    assert len(sel) == 5 and stats['frozen'] is True


def test_live_frames_not_frozen():
    buf = FrameBuffer()
    for i in range(5):
        buf.add(int((3.0 + 0.1 * i) * S), 1000 + i, payload=i, now_ns=int((3.0 + 0.1 * i) * S))
    _, stats = select_burst(buf.frames(), int(2.9 * S), int(3.5 * S), int(0.2 * S), 5)
    assert stats['frozen'] is False


def test_buffer_is_bounded():
    buf = FrameBuffer(maxlen=10)
    for i in range(50):
        buf.add(i * S // 10, i, payload=i, now_ns=i * S // 10)
    assert len(buf.frames()) == 10
