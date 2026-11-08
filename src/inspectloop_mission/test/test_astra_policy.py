"""Astra F2 (measurement contract before ACCEPT) and the policy side of F1."""
import math

import pytest

from inspectloop_mission import decision_policy as dp

P = dp.PolicyParams(max_sigma=0.15, min_valid_frames=3, max_retries_same=1, max_observations=8,
                    scale_min=0.0, scale_max=10.0, out_of_scale_tol=0.05)


def obs(**kw):
    base = {'has_value': True, 'value': 4.2, 'sigma': 0.06, 'frames_valid': 5,
            'frame_spread': 0.02, 'quality_flags': []}
    base.update(kw)
    return base


def test_reference_observation_is_accepted():
    assert dp.decide(obs(), 0, 2, 1, P).decision == dp.ACCEPT


@pytest.mark.parametrize('field,bad', [
    ('value', None), ('value', float('nan')), ('value', float('inf')), ('value', -float('inf')),
    ('value', '4.2'), ('value', True),
    ('sigma', None), ('sigma', float('nan')), ('sigma', float('inf')), ('sigma', -0.05),
    ('sigma', 0.0), ('sigma', '0.05'),
    ('frames_valid', None), ('frames_valid', float('nan')), ('frames_valid', '5'),
    ('frames_valid', -1),
    ('has_value', False), ('has_value', 'yes'),
    ('quality_flags', None), ('quality_flags', 'OCCLUDED'),
])
def test_malformed_measurement_is_never_accepted(field, bad):
    d = dp.decide(obs(**{field: bad}), 0, 2, 1, P)
    assert d.decision != dp.ACCEPT, (field, bad, d.reason)


def test_malformed_measurement_retries_then_gives_up_without_moving():
    first = dp.decide(obs(sigma=float('nan')), 0, 2, 1, P)
    assert first.decision == dp.RETRY_SAME
    second = dp.decide(obs(sigma=float('nan')), 1, 2, 2, P)
    assert second.decision == dp.UNRESOLVED
    assert 'invalid measurement' in second.reason


@pytest.mark.parametrize('value', [-0.1, -0.06, 10.06, 10.1, 25.0])
def test_out_of_scale_value_without_flag_is_not_accepted(value):
    """Defence in depth: the policy checks the physical range itself, the reader's flag
    is not the only line of defence."""
    d = dp.decide(obs(value=value), 0, 2, 1, P)
    assert d.decision != dp.ACCEPT


@pytest.mark.parametrize('value', [0.0, -0.04, 10.0, 10.04])
def test_values_within_scale_tolerance_are_accepted(value):
    assert dp.decide(obs(value=value), 0, 2, 1, P).decision == dp.ACCEPT


def test_missing_dict_is_not_accepted():
    assert dp.decide({}, 0, 2, 1, P).decision != dp.ACCEPT
    assert dp.decide(None, 0, 2, 1, P).decision != dp.ACCEPT


@pytest.mark.parametrize('flag', ['AMBIGUOUS_NEEDLE', 'NEEDLE_PARTIAL'])
def test_needle_evidence_flags_are_viewpoint_problems(flag):
    """A competing line / hidden needle is direction dependent -> look from the other side."""
    d = dp.decide(obs(quality_flags=[flag]), 0, 2, 1, P)
    assert d.decision == dp.MOVE_NEW and d.hint == 'different_side'


def test_unknown_flag_blocks_acceptance():
    d = dp.decide(obs(quality_flags=['SOMETHING_NEW']), 0, 2, 1, P)
    assert d.decision != dp.ACCEPT


def test_stale_observation_flag_is_transient():
    d = dp.decide(obs(quality_flags=['STALE_OBSERVATION'], has_value=False, value=None), 0, 2, 1, P)
    assert d.decision == dp.RETRY_SAME


def test_robot_moving_flag_is_transient():
    d = dp.decide(obs(quality_flags=['ROBOT_MOVING']), 0, 2, 1, P)
    assert d.decision == dp.RETRY_SAME


def test_sigma_exactly_at_limit_is_accepted_and_just_above_is_not():
    assert dp.decide(obs(sigma=0.15), 0, 2, 1, P).decision == dp.ACCEPT
    assert dp.decide(obs(sigma=math.nextafter(0.15, 1.0)), 0, 2, 1, P).decision != dp.ACCEPT


@pytest.mark.parametrize('sensor_flag', ['FROZEN_FRAMES', 'CLOCK_RESET'])
def test_sensor_invalidation_dominates_derived_flags(sensor_flag):
    """Found in the Gazebo frozen-camera test: a frozen burst also yields TAG_NOT_VISIBLE, which
    alone would justify moving. Flags computed from invalid data are meaningless -> retry once,
    then UNRESOLVED, never MOVE_NEW."""
    o = obs(has_value=False, value=None, sigma=None, frames_valid=0,
            quality_flags=['TAG_NOT_VISIBLE', sensor_flag])
    assert dp.decide(o, 0, 2, 1, P).decision == dp.RETRY_SAME
    d = dp.decide(o, 1, 2, 2, P)
    assert d.decision == dp.UNRESOLVED
