import pytest

from inspectloop_mission import decision_policy as dp

P = dp.PolicyParams(max_sigma=0.15, min_valid_frames=3, max_retries_same=1, max_observations=8)


def obs(flags=(), value=4.2, sigma=0.06, valid=5, spread=0.02):
    return {'has_value': value is not None, 'value': value, 'sigma': sigma, 'frames_valid': valid,
            'frame_spread': spread, 'quality_flags': list(flags)}


def test_accept_clean_reading():
    d = dp.decide(obs(), 0, 2, 1, P)
    assert d.decision == dp.ACCEPT


def test_accept_with_intermittent_tag_if_enough_frames():
    assert dp.decide(obs(['TAG_INTERMITTENT'], valid=4), 0, 2, 1, P).decision == dp.ACCEPT


def test_high_sigma_without_flags_moves_closer():
    d = dp.decide(obs(sigma=0.3, spread=0.02), 0, 2, 1, P)
    assert d.decision == dp.MOVE_NEW and d.hint == 'closer'


def test_high_sigma_from_spread_retries_first():
    d = dp.decide(obs(sigma=0.3, spread=0.4), 0, 2, 1, P)
    assert d.decision == dp.RETRY_SAME


@pytest.mark.parametrize('flag,hint', [('OCCLUDED', 'different_side'),
                                       ('OBLIQUE_VIEW', 'frontal'), ('TOO_FAR', 'closer'),
                                       ('LOW_NEEDLE_CONTRAST', 'different_side')])
def test_geometric_problem_moves(flag, hint):
    d = dp.decide(obs([flag]), 0, 2, 1, P)
    assert d.decision == dp.MOVE_NEW and d.hint == hint


def test_geometric_problem_without_viewpoints_is_unresolved():
    assert dp.decide(obs(['OCCLUDED']), 0, 0, 3, P).decision == dp.UNRESOLVED


@pytest.mark.parametrize('flag', ['TAG_NOT_VISIBLE', 'INCONSISTENT_FRAMES', 'BLURRED', 'NO_IMAGE'])
def test_transient_problem_retries_once_then_escalates(flag):
    first = dp.decide(obs([flag], value=None, valid=0), 0, 2, 1, P)
    assert first.decision == dp.RETRY_SAME
    second = dp.decide(obs([flag], value=None, valid=0), 1, 2, 2, P)
    if flag == 'NO_IMAGE':
        assert second.decision == dp.UNRESOLVED          # moving cannot fix a camera fault
    else:
        assert second.decision == dp.MOVE_NEW
        if flag == 'TAG_NOT_VISIBLE':
            assert second.hint == 'different_side'


def test_persistent_tag_not_visible_with_no_viewpoint_left_is_unresolved():
    d = dp.decide(obs(['TAG_NOT_VISIBLE'], value=None, valid=0), 1, 0, 6, P)
    assert d.decision == dp.UNRESOLVED


def test_observation_budget_forces_unresolved():
    d = dp.decide(obs(['OCCLUDED']), 0, 2, 8, P)
    assert d.decision == dp.UNRESOLVED


def test_too_few_valid_frames_retries():
    d = dp.decide(obs(valid=2), 0, 2, 1, P)
    assert d.decision == dp.RETRY_SAME


def test_mixed_flags_geometric_wins():
    d = dp.decide(obs(['OCCLUDED', 'INCONSISTENT_FRAMES']), 0, 2, 1, P)
    assert d.decision == dp.MOVE_NEW


def test_decision_codes_match_message_constants():
    assert [dp.DECISION_CODES[k] for k in (dp.ACCEPT, dp.RETRY_SAME, dp.MOVE_NEW,
                                           dp.UNRESOLVED)] == [0, 1, 2, 3]
