"""V3 (Astra V2 review, F5): result age, stamp ordering, clock epochs and odometry freshness."""
import math

from inspectloop_mission.observation_guard import ExpectedObservation, is_settled, validate_observation


def expected(**kw):
    e = ExpectedObservation(observation_id='m1/PG-11/V0/1', viewpoint_epoch=3,
                            request_time_s=100.0, now_s=101.5, x=-7.0, y=2.49,
                            yaw=math.pi / 2)
    for k, v in kw.items():
        setattr(e, k, v)
    return e


def reading(**kw):
    r = {'observation_id': 'm1/PG-11/V0/1', 'viewpoint_epoch': 3, 'first_frame_stamp_s': 100.6,
         'last_frame_stamp_s': 101.0, 'pose_known': True, 'acq_x': -6.95, 'acq_y': 2.52,
         'acq_yaw': math.pi / 2 + 0.1, 'pose_motion_m': 0.002, 'pose_motion_rad': 0.003,
         'frames_received': 5, 'has_value': True}
    r.update(kw)
    return r


def test_reference_reading_is_admissible():
    assert validate_observation(reading(), expected()) == []


def test_astra_23_second_old_result_is_rejected():
    """Exact review case: frames 100.6-101.0 s, consumed at 124.0 s (23 s after the last frame)."""
    p = validate_observation(reading(), expected(now_s=124.0))
    assert any('old' in x for x in p), p


def test_astra_0_8_second_rollback_between_request_and_consumption_is_rejected():
    """Exact review case: the sim clock rolled back by 0.8 s after the request and then ran on,
    so now > request again - only the clock epoch reveals the discontinuity."""
    p = validate_observation(reading(), expected(now_s=101.5, clock_epoch_request=4,
                                                 clock_epoch_now=5))
    assert any('clock' in x for x in p), p


def test_first_frame_after_last_frame_is_rejected():
    p = validate_observation(reading(first_frame_stamp_s=101.2, last_frame_stamp_s=100.7),
                             expected())
    assert any('order' in x for x in p), p


def test_value_without_any_frame_is_rejected():
    p = validate_observation(reading(frames_received=0), expected())
    assert p


def test_value_without_acquisition_pose_is_rejected():
    assert validate_observation(reading(pose_known=False), expected())


def test_reply_from_another_mission_is_rejected():
    p = validate_observation(reading(observation_id='m0/PG-11/V0/1'),
                             expected(mission_id='m1'))
    assert p


def test_result_age_limit_is_configurable_and_inclusive():
    assert validate_observation(reading(), expected(now_s=109.0, max_result_age_s=8.0)) == []
    assert validate_observation(reading(), expected(now_s=109.01, max_result_age_s=8.0))


def test_stale_odometry_cannot_prove_the_robot_is_settled():
    """Samples show rest, but the newest is 10 s old: the odometry stream may be dead."""
    samples = [(40.0 + 0.05 * i, 0.0, 0.0) for i in range(200)]      # 40.0 .. 49.95
    assert not is_settled(samples, 0.5, 0.02, 0.05, now_s=60.0, max_age_s=0.3)
    assert is_settled(samples, 0.5, 0.02, 0.05, now_s=50.1, max_age_s=0.3)


def test_future_dated_odometry_is_not_trusted():
    samples = [(50.0 + 0.05 * i, 0.0, 0.0) for i in range(20)]       # up to 50.95
    assert not is_settled(samples, 0.5, 0.02, 0.05, now_s=45.0, max_age_s=0.3)


def test_sparse_odometry_window_is_not_enough():
    samples = [(49.0, 0.0, 0.0), (50.0, 0.0, 0.0)]                   # one sample in the window
    assert not is_settled(samples, 0.5, 0.02, 0.05, now_s=50.05, max_age_s=0.3)
