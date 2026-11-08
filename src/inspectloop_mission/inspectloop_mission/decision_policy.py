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
from typing import Dict, Iterable, List

ACCEPT, RETRY_SAME, MOVE_NEW, UNRESOLVED = 'ACCEPT', 'RETRY_SAME', 'MOVE_NEW', 'UNRESOLVED'
DECISION_CODES = {ACCEPT: 0, RETRY_SAME: 1, MOVE_NEW: 2, UNRESOLVED: 3}

TRANSIENT_FLAGS = {'NO_IMAGE', 'TAG_INTERMITTENT', 'INCONSISTENT_FRAMES', 'BLURRED',
                   'TAG_NOT_VISIBLE', 'STALE_OBSERVATION', 'ROBOT_MOVING', 'FROZEN_FRAMES',
                   'CLOCK_RESET', 'INVALID_MEASUREMENT', 'WRONG_VIEWPOINT'}
GEOMETRIC_FLAGS = {'OCCLUDED', 'OBLIQUE_VIEW', 'TOO_FAR', 'LOW_NEEDLE_CONTRAST', 'OUT_OF_SCALE',
                   'AMBIGUOUS_NEEDLE', 'NEEDLE_PARTIAL'}
# problems that moving the robot cannot fix (sensor / software): retry, then give up
NOT_FIXABLE_BY_MOVING = {'NO_IMAGE', 'FROZEN_FRAMES', 'CLOCK_RESET', 'INVALID_MEASUREMENT'}
# flags that do not by themselves invalidate a reading with enough valid frames
INFORMATIONAL_FLAGS = {'TAG_INTERMITTENT'}


@dataclass
class PolicyParams:
    max_sigma: float = 0.15              # bar (1 sigma) - acceptance limit (heuristic sigma)
    min_valid_frames: int = 3
    max_retries_same: int = 1            # per viewpoint
    max_observations: int = 8            # per gauge (all viewpoints)
    scale_min: float = 0.0               # printed scale of the gauge
    scale_max: float = 10.0
    out_of_scale_tol: float = 0.05       # beyond this the value is physically unsupported


@dataclass
class Decision:
    decision: str
    reason: str
    hint: str = 'any'

    @property
    def code(self) -> int:
        return DECISION_CODES[self.decision]


def _is_real(x) -> bool:
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def contract_violations(obs, p: PolicyParams) -> List[str]:
    """Check the measurement contract required before ACCEPT (Astra F2). Never clamps."""
    if not isinstance(obs, dict):
        return ['observation is not a mapping']
    errs = []
    if obs.get('has_value') is not True:
        errs.append('has_value is not True')
    v, s, n = obs.get('value'), obs.get('sigma'), obs.get('frames_valid')
    if not _is_real(v):
        errs.append(f'value {v!r} is not a finite number')
    if not _is_real(s) or s <= 0.0:
        errs.append(f'sigma {s!r} is not a finite positive number')
    if not (isinstance(n, int) and not isinstance(n, bool)) or n < 0:
        errs.append(f'frames_valid {n!r} is not a non-negative integer')
    if not isinstance(obs.get('quality_flags'), (list, tuple)):
        errs.append('quality_flags is not a list')
    if _is_real(v) and not (p.scale_min - p.out_of_scale_tol <= v <= p.scale_max + p.out_of_scale_tol):
        errs.append(f'value {v} outside the printed scale [{p.scale_min}, {p.scale_max}]')
    return errs


def _hint_for(flags: Iterable[str]) -> str:
    flags = set(flags)
    if flags & {'OCCLUDED', 'TAG_NOT_VISIBLE', 'LOW_NEEDLE_CONTRAST', 'AMBIGUOUS_NEEDLE',
                'NEEDLE_PARTIAL'}:
        return 'different_side'
    if 'OBLIQUE_VIEW' in flags:
        return 'frontal'
    if 'TOO_FAR' in flags:
        return 'closer'
    return 'any'


def decide(obs: Dict, retries_at_view: int, viewpoints_remaining: int, observations_used: int,
           p: PolicyParams = PolicyParams()) -> Decision:
    """obs: dict with has_value, value, sigma, frames_valid, frame_spread, quality_flags."""
    budget_left = observations_used < p.max_observations
    can_move = viewpoints_remaining > 0 and budget_left
    can_retry = retries_at_view < p.max_retries_same and budget_left

    raw_flags = obs.get('quality_flags') if isinstance(obs, dict) else None
    flags = set(raw_flags) if isinstance(raw_flags, (list, tuple)) else set()
    errs = contract_violations(obs, p)
    no_value = not isinstance(obs, dict) or obs.get('has_value') is not True

    # 1. a reading may only be accepted if it satisfies the full contract and is unflagged
    blocking = flags - INFORMATIONAL_FLAGS
    if not errs and not blocking:
        valid = obs['frames_valid']
        sigma = obs['sigma']
        if valid >= p.min_valid_frames and sigma <= p.max_sigma:
            return Decision(ACCEPT, f'{valid} valid frames, sigma {sigma:.3f} <= {p.max_sigma}')

    # 2. physically out of scale -> a geometric problem (misread from this view), not accepted
    out_of_scale = any('outside the printed scale' in e for e in errs)
    if out_of_scale:
        flags = flags | {'OUT_OF_SCALE'}
        blocking = flags - INFORMATIONAL_FLAGS

    # 3. malformed measurement with no explanation in the flags -> software/sensor problem
    structural = [e for e in errs if 'outside the printed scale' not in e]
    if structural and not blocking and not no_value:
        if can_retry:
            return Decision(RETRY_SAME, 'invalid measurement (' + '; '.join(structural) + ')')
        return Decision(UNRESOLVED, 'invalid measurement persisted: ' + '; '.join(structural))
    if structural and not blocking and no_value:
        blocking = {'INVALID_MEASUREMENT'} if isinstance(obs, dict) and obs.get('has_value') is not False \
            else {'TOO_FEW_VALID_FRAMES'}

    geometric = blocking & GEOMETRIC_FLAGS
    transient = blocking & TRANSIENT_FLAGS
    unknown = blocking - GEOMETRIC_FLAGS - TRANSIENT_FLAGS

    if not blocking and not errs:
        sigma = obs['sigma']
        valid = obs['frames_valid']
        if valid < p.min_valid_frames:
            transient = {'TOO_FEW_VALID_FRAMES'}
        elif sigma > p.max_sigma:
            spread = obs.get('frame_spread')
            spread = float(spread) if _is_real(spread) else 0.0
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

    transient = transient | unknown          # unknown flags: never accepted, treated cautiously
    why = ', '.join(sorted(transient)) or 'unreliable observation'
    if can_retry:
        return Decision(RETRY_SAME, f'possibly transient ({why}): observe again from the same pose')
    if transient and transient <= NOT_FIXABLE_BY_MOVING:
        if transient == {'NO_IMAGE'}:
            return Decision(UNRESOLVED, 'camera delivers no frames: moving cannot fix a sensor fault')
        return Decision(UNRESOLVED, f'{why} persisted: moving cannot fix it')
    if can_move:
        return Decision(MOVE_NEW, f'{why} persisted after {retries_at_view} retry(ies)',
                        _hint_for(transient))
    return Decision(UNRESOLVED, f'{why} persisted and no viewpoint/observation budget left')
