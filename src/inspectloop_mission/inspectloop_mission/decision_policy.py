"""Uncertainty-aware decision policy for one gauge observation (ROS independent).

    ACCEPT      the reading is reliable: enough valid frames, no quality flag that invalidates
                it, and sigma <= max_sigma.
    RETRY_SAME  the problem looks *transient* (missing/intermittent frames, frame-to-frame
                disagreement, blur, or a tag that is not visible - which may be a passer-by)
                -> observe again from the same pose, at most max_retries_same times per pose.
    MOVE_NEW    the problem is *geometric*: the dial is occluded, seen too obliquely, too far
                away / too few pixels (sigma too large), or the needle contrast is poor from
                this direction -> ask the viewpoint planner for a different pose and navigate.
                A persisting transient-looking problem also escalates to MOVE_NEW.
    UNRESOLVED  no reliable reading is possible within the budget (no viewpoint left,
                observation budget spent, or a camera fault that moving cannot fix).
                The mission reports this honestly instead of publishing a guess.

The policy also returns a *hint* for the viewpoint planner:
    'different_side' (occlusion / tag hidden), 'frontal' (too oblique), 'closer' (too far,
    sigma dominated by resolution), 'any'.
"""
from dataclasses import dataclass
import math
from typing import Dict, Iterable

ACCEPT, RETRY_SAME, MOVE_NEW, UNRESOLVED = 'ACCEPT', 'RETRY_SAME', 'MOVE_NEW', 'UNRESOLVED'
DECISION_CODES = {ACCEPT: 0, RETRY_SAME: 1, MOVE_NEW: 2, UNRESOLVED: 3}

TRANSIENT_FLAGS = {'NO_IMAGE', 'TAG_INTERMITTENT', 'INCONSISTENT_FRAMES', 'BLURRED',
                   'TAG_NOT_VISIBLE'}
GEOMETRIC_FLAGS = {'OCCLUDED', 'OBLIQUE_VIEW', 'TOO_FAR', 'LOW_NEEDLE_CONTRAST', 'OUT_OF_SCALE'}
# flags that do not by themselves invalidate a reading with enough valid frames
INFORMATIONAL_FLAGS = {'TAG_INTERMITTENT'}


@dataclass
class PolicyParams:
    max_sigma: float = 0.15              # bar (1 sigma) - acceptance limit
    min_valid_frames: int = 3
    max_retries_same: int = 1            # per viewpoint
    max_observations: int = 8            # per gauge (all viewpoints)


@dataclass
class Decision:
    decision: str
    reason: str
    hint: str = 'any'

    @property
    def code(self) -> int:
        return DECISION_CODES[self.decision]


def _hint_for(flags: Iterable[str]) -> str:
    flags = set(flags)
    if flags & {'OCCLUDED', 'TAG_NOT_VISIBLE', 'LOW_NEEDLE_CONTRAST'}:
        return 'different_side'
    if 'OBLIQUE_VIEW' in flags:
        return 'frontal'
    if 'TOO_FAR' in flags:
        return 'closer'
    return 'any'


def decide(obs: Dict, retries_at_view: int, viewpoints_remaining: int, observations_used: int,
           p: PolicyParams = PolicyParams()) -> Decision:
    """obs: dict with has_value, value, sigma, frames_valid, frame_spread, quality_flags."""
    flags = set(obs.get('quality_flags') or [])
    sigma = obs.get('sigma')
    sigma = float('nan') if sigma is None else float(sigma)
    valid = int(obs.get('frames_valid', 0))
    has_value = bool(obs.get('has_value'))

    blocking = flags - INFORMATIONAL_FLAGS
    if has_value and not blocking and valid >= p.min_valid_frames and sigma <= p.max_sigma:
        return Decision(ACCEPT, f'{valid} valid frames, sigma {sigma:.3f} <= {p.max_sigma}')

    budget_left = observations_used < p.max_observations
    can_move = viewpoints_remaining > 0 and budget_left
    can_retry = retries_at_view < p.max_retries_same and budget_left

    geometric = blocking & GEOMETRIC_FLAGS
    transient = blocking & TRANSIENT_FLAGS

    if not blocking:
        # no explicit flag, but the reading is still not good enough
        if valid < p.min_valid_frames:
            transient = {'TOO_FEW_VALID_FRAMES'}
        elif not math.isnan(sigma) and sigma > p.max_sigma:
            spread = float(obs.get('frame_spread') or 0.0)
            if spread > p.max_sigma and can_retry:
                return Decision(RETRY_SAME, f'sigma {sigma:.3f} dominated by frame spread '
                                            f'{spread:.3f}: re-observe')
            geometric = {'HIGH_UNCERTAINTY'}

    if geometric:
        why = ', '.join(sorted(geometric))
        if can_move:
            hint = 'closer' if geometric == {'HIGH_UNCERTAINTY'} else _hint_for(geometric)
            return Decision(MOVE_NEW, f'viewpoint-dependent problem ({why})', hint)
        return Decision(UNRESOLVED, f'{why} and no viewpoint/observation budget left')

    why = ', '.join(sorted(transient)) or 'unreliable observation'
    if can_retry:
        return Decision(RETRY_SAME, f'possibly transient ({why}): observe again from the same pose')
    if flags == {'NO_IMAGE'}:
        return Decision(UNRESOLVED, 'camera delivers no frames: moving cannot fix a sensor fault')
    if can_move:
        return Decision(MOVE_NEW, f'{why} persisted after {retries_at_view} retry(ies)',
                        _hint_for(transient))
    return Decision(UNRESOLVED, f'{why} persisted and no viewpoint/observation budget left')
