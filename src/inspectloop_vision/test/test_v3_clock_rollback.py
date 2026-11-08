"""V3 (Astra V2 review, F5): any backward clock step is a discontinuity (no 1 s threshold)."""
from inspectloop_vision.frame_buffer import FrameBuffer, select_burst

S = 1_000_000_000


def _fill(buf, t0, n=10, dt=0.1):
    for i in range(n):
        t = int((t0 + i * dt) * S)
        buf.add(t, i, f'f{i}', t)


def test_astra_0_8_second_rollback_invalidates_previous_frames():
    buf = FrameBuffer(maxlen=30)
    _fill(buf, 9.1)                                   # frames 9.1 .. 10.0 s, clock at 10.0
    epoch = buf.time_epoch
    reset = buf.check_clock(int(9.2 * S))             # sim clock rolled back by 0.8 s
    assert reset
    assert buf.time_epoch == epoch + 1
    sel, _ = select_burst(buf.frames(), int(9.25 * S), int(9.4 * S), int(0.2 * S), 5)
    assert sel == []                                  # nothing from the old timeline


def test_small_rollback_of_frame_stamps_is_a_reset():
    buf = FrameBuffer(maxlen=30)
    _fill(buf, 20.0)
    assert buf.add(int(20.5 * S), 99, 'old-timeline', int(20.95 * S)) is True


def test_monotonic_frames_never_reset():
    buf = FrameBuffer(maxlen=30)
    _fill(buf, 1.0, n=30, dt=0.033)
    assert buf.time_epoch == 0


def test_explicit_jump_notification_clears_the_buffer():
    buf = FrameBuffer(maxlen=30)
    _fill(buf, 5.0)
    buf.notify_clock_jump()
    assert buf.frames() == [] and buf.time_epoch == 1
