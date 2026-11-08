"""V3 (Astra V2 review, F7): system faults are distinguished from image-quality failures."""
import pytest

from inspectloop_mission import decision_policy as dp

P = dp.PolicyParams(max_sigma=0.15, min_valid_frames=3, max_retries_same=1, max_observations=8)


def fault(*flags):
    return {'has_value': False, 'value': None, 'sigma': None, 'frames_valid': 0,
            'frame_spread': None, 'quality_flags': list(flags)}


@pytest.mark.parametrize('flag', ['PROCESSING_ERROR', 'SERVICE_FAILURE'])
def test_system_fault_retries_then_unresolved_as_system_fault_never_moves(flag):
    assert dp.decide(fault(flag), 0, 2, 1, P).decision == dp.RETRY_SAME
    d = dp.decide(fault(flag), 1, 2, 2, P)
    assert d.decision == dp.UNRESOLVED
    assert 'system fault' in d.reason and 'not an image-quality problem' in d.reason


def test_invalid_frames_are_a_data_problem_not_occlusion():
    d = dp.decide(fault('INVALID_FRAMES'), 1, 2, 2, P)
    assert d.decision == dp.UNRESOLVED and 'invalid image data' in d.reason


def test_camera_unavailable_message_kept():
    d = dp.decide(fault('NO_IMAGE'), 1, 2, 2, P)
    assert d.decision == dp.UNRESOLVED and 'camera delivers no frames' in d.reason


def test_occlusion_still_moves():
    assert dp.decide(fault('OCCLUDED'), 0, 2, 1, P).decision == dp.MOVE_NEW


@pytest.mark.parametrize('status,flags,nav,expected', [
    ('ACCEPTED', [], False, 'MEASURED'),
    ('UNRESOLVED', ['PROCESSING_ERROR'], False, 'SYSTEM_FAULT'),
    ('UNRESOLVED', ['SERVICE_FAILURE'], False, 'SYSTEM_FAULT'),
    ('UNRESOLVED', ['NO_IMAGE'], False, 'CAMERA_UNAVAILABLE'),
    ('UNRESOLVED', ['FROZEN_FRAMES', 'TAG_NOT_VISIBLE'], False, 'IMAGE_INVALID'),
    ('UNRESOLVED', ['INVALID_FRAMES'], False, 'IMAGE_INVALID'),
    ('UNRESOLVED', ['STALE_OBSERVATION'], False, 'IMAGE_INVALID'),
    ('UNRESOLVED', ['OCCLUDED', 'TAG_NOT_VISIBLE'], False, 'INSUFFICIENT_VISUAL_EVIDENCE'),
    ('UNRESOLVED', ['AMBIGUOUS_NEEDLE'], False, 'INSUFFICIENT_VISUAL_EVIDENCE'),
    ('UNRESOLVED', [], True, 'NAVIGATION_FAILURE'),
])
def test_outcome_class(status, flags, nav, expected):
    assert dp.outcome_class(status, flags, nav) == expected
    assert expected in dp.OUTCOME_CLASSES
