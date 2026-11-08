"""Astra F5 (mission side): the mission re-validates every returned observation."""
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
         'frames_received': 5}
    r.update(kw)
    return r


def test_valid_observation_passes():
    assert validate_observation(reading(), expected()) == []


def test_wrong_observation_id_rejected():
    assert validate_observation(reading(observation_id='m1/PG-11/V0/0'), expected())


def test_previous_viewpoint_epoch_rejected():
    assert validate_observation(reading(viewpoint_epoch=2), expected())


def test_frame_older_than_request_rejected():
    assert validate_observation(reading(first_frame_stamp_s=99.5), expected())


def test_future_dated_frame_rejected():
    assert validate_observation(reading(last_frame_stamp_s=150.0), expected())


def test_clock_reset_between_request_and_result_rejected():
    assert validate_observation(reading(), expected(now_s=5.0))


def test_unknown_acquisition_pose_rejected():
    assert validate_observation(reading(pose_known=False), expected())


def test_acquired_at_wrong_place_rejected():
    assert validate_observation(reading(acq_x=-5.0), expected())


def test_acquired_with_wrong_heading_rejected():
    assert validate_observation(reading(acq_yaw=math.pi / 2 + 0.8), expected())


def test_robot_moving_during_burst_rejected():
    assert validate_observation(reading(pose_motion_m=0.08), expected())


def test_no_frames_is_not_a_freshness_problem():
    """NO_IMAGE results carry no stamps; freshness checks apply only to captured frames."""
    assert validate_observation(reading(frames_received=0, first_frame_stamp_s=0.0,
                                        last_frame_stamp_s=0.0, pose_known=False),
                                expected()) == []


def test_settled():
    still = [(t * 0.05, 0.001, 0.002) for t in range(20)]
    assert is_settled(still, window_s=0.5, max_v=0.02, max_w=0.05)
    moving = still[:-3] + [(0.9, 0.2, 0.0), (0.95, 0.2, 0.0), (1.0, 0.2, 0.0)]
    assert not is_settled(moving, window_s=0.5, max_v=0.02, max_w=0.05)
    assert not is_settled([(0.0, 0.0, 0.0)], window_s=0.5, max_v=0.02, max_w=0.05)
