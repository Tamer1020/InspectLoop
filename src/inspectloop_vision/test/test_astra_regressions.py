"""Regression tests for the Astra audit findings F1 (wrong readings accepted) and F2
(invalid numerical values accepted) on the vision side.

Textures are rendered with the *same* generator that builds the Gazebo world
(make_gauge_texture), so needle values, including out-of-scale ones, are exact.
The safety property under test is: **a reading is never reported clean (no quality flag)
while being wrong**. A reader may refuse to read (flag) — it may not silently misread.
"""
import importlib.util
import math
import os

import cv2
import numpy as np
import pytest

from inspectloop_vision import gauge_reader as gr

HERE = os.path.dirname(os.path.abspath(__file__))
WRONG_TOL = 0.10          # bar: |error| above this with no flag = incorrect clean reading


def _load_generator():
    candidates = [os.path.join(HERE, '..', '..', 'inspectloop_gazebo', 'scripts',
                               'generate_warehouse.py')]
    try:
        from ament_index_python.packages import get_package_prefix
        candidates.append(os.path.join(get_package_prefix('inspectloop_gazebo'), 'lib',
                                       'inspectloop_gazebo', 'generate_warehouse.py'))
    except Exception:  # noqa: BLE001
        pass
    for c in candidates:
        if os.path.exists(c):
            spec = importlib.util.spec_from_file_location('generate_warehouse', c)
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return mod
    pytest.skip('gauge texture generator not found')


GEN = _load_generator()


def tex_polar(r_frac, ang_deg):
    a = math.radians(ang_deg)
    return (gr.DIAL_CENTER[0] + r_frac * gr.DIAL_R * math.sin(a),
            gr.DIAL_CENTER[1] - r_frac * gr.DIAL_R * math.cos(a))


def gauge(value, tag=11):
    return GEN.make_gauge_texture(tag, f'PG-{tag}', value)


def draw_radial(tex, ang_deg, r0=0.12, r1=0.80, width=6, color=(15, 15, 15)):
    p0 = tuple(int(round(v)) for v in tex_polar(r0, ang_deg))
    p1 = tuple(int(round(v)) for v in tex_polar(r1, ang_deg))
    cv2.line(tex, p0, p1, color, width, cv2.LINE_AA)
    return tex


def draw_wedge(tex, a0, a1, r0, r1, color):
    pts = [tex_polar(r0, a) for a in np.linspace(a0, a1, 20)] + \
          [tex_polar(r1, a) for a in np.linspace(a1, a0, 20)]
    cv2.fillPoly(tex, [np.array(pts, np.int32)], color, cv2.LINE_AA)
    return tex


def render(tex, scale=0.32, skew=0.0, noise=2.0, seed=0):
    scene = np.full((480, 640, 3), (110, 120, 130), np.uint8)
    h, w = tex.shape[:2]
    cx, cy = 320, 245
    pw, ph = w * scale, h * scale
    k = skew * ph
    dst = np.float32([[cx - pw / 2, cy - ph / 2 - k / 2], [cx + pw / 2, cy - ph / 2 + k / 2],
                      [cx + pw / 2, cy + ph / 2 - k / 2], [cx - pw / 2, cy + ph / 2 + k / 2]])
    H = cv2.getPerspectiveTransform(np.float32([[0, 0], [w, 0], [w, h], [0, h]]), dst)
    m = cv2.warpPerspective(np.full((h, w), 255, np.uint8), H, (640, 480))
    scene[m > 0] = cv2.warpPerspective(tex, H, (640, 480))[m > 0]
    out = scene.astype(np.float32) + np.random.default_rng(seed).normal(0, noise, scene.shape)
    return np.clip(out, 0, 255).astype(np.uint8)


@pytest.fixture(scope='module')
def reader():
    return gr.GaugeReader()


def observe(reader, tex, n=5, **kw):
    frames = [reader.read_frame(render(tex, seed=s, **kw), 11) for s in range(n)]
    return gr.fuse(frames, n)


def assert_not_silently_wrong(obs, truth):
    clean = obs.has_value and not obs.flags
    if clean:
        assert abs(obs.value - truth) <= WRONG_TOL, (
            f'clean reading {obs.value:.3f} but truth {truth} (sigma {obs.sigma:.3f})')


# ------------------------------------------------------------------ F1: distractors / ambiguity
TRUTH = 4.2
NEEDLE_ANG = gr.value_to_angle(TRUTH)


@pytest.mark.parametrize('offset', [8.0, -8.0, 5.0, 12.0, 20.0, -35.0])
def test_f1_radial_distractor_near_needle_is_never_silently_wrong(reader, offset):
    """Astra F1: a dark radial line 8 deg from the needle was accepted as 4.508 bar (truth 4.2)."""
    tex = draw_radial(gauge(TRUTH), NEEDLE_ANG + offset)
    assert_not_silently_wrong(observe(reader, tex), TRUTH)


@pytest.mark.parametrize('offset', [8.0, -8.0, 20.0])
def test_f1_equally_dark_second_line_is_flagged_ambiguous(reader, offset):
    """Two needle-like lines of equal strength cannot be disambiguated: must not be clean."""
    tex = draw_radial(gauge(TRUTH), NEEDLE_ANG + offset, width=7)
    obs = observe(reader, tex)
    assert obs.flags, obs.detail


def test_f1_white_occluder_over_needle_tip(reader):
    tex = draw_wedge(gauge(TRUTH), NEEDLE_ANG - 12, NEEDLE_ANG + 12, 0.40, 0.90, (250, 250, 250))
    assert_not_silently_wrong(observe(reader, tex), TRUTH)


def test_f1_white_occluder_hiding_most_of_needle(reader):
    tex = draw_wedge(gauge(TRUTH), NEEDLE_ANG - 15, NEEDLE_ANG + 15, 0.12, 0.90, (250, 250, 250))
    obs = observe(reader, tex)
    assert_not_silently_wrong(obs, TRUTH)
    assert obs.flags, 'needle invisible but reading reported clean'


def test_f1_non_radial_line_crossing_dial(reader):
    """A cable/crack across the face (not through the pivot) near the needle."""
    tex = gauge(TRUTH)
    p0 = tuple(int(v) for v in tex_polar(0.75, NEEDLE_ANG - 40))
    p1 = tuple(int(v) for v in tex_polar(0.60, NEEDLE_ANG + 50))
    cv2.line(tex, p0, p1, (20, 20, 20), 7, cv2.LINE_AA)
    assert_not_silently_wrong(observe(reader, tex), TRUTH)


def test_f1_dark_shadow_sector_adjacent_to_needle(reader):
    tex = draw_wedge(gauge(TRUTH), NEEDLE_ANG + 4, NEEDLE_ANG + 14, 0.15, 0.60, (70, 70, 70))
    assert_not_silently_wrong(observe(reader, tex), TRUTH)


@pytest.mark.parametrize('value', [0.4, 2.0, 4.2, 5.0, 6.8, 8.7, 9.6])
def test_f1_clean_readings_remain_accurate(reader, value):
    obs = observe(reader, gauge(value))
    assert obs.has_value and obs.flags == [], obs.detail
    assert abs(obs.value - value) < 0.06, obs.detail


@pytest.mark.parametrize('value,skew,scale', [(3.3, 0.2, 0.32), (7.1, -0.25, 0.5),
                                              (1.2, 0.1, 0.24)])
def test_f1_clean_perspective_readings_remain_accurate(reader, value, skew, scale):
    obs = observe(reader, gauge(value), skew=skew, scale=scale)
    assert obs.has_value and obs.flags == [], obs.detail
    assert abs(obs.value - value) < 0.08, obs.detail


# ------------------------------------------------------------------ F2: physical validity
@pytest.mark.parametrize('value', [-0.1, -0.3, 10.1, 10.4])
def test_f2_out_of_scale_needle_is_flagged_and_not_clamped(reader, value):
    obs = observe(reader, gauge(value))
    assert 'OUT_OF_SCALE' in obs.flags, obs.detail
    assert obs.has_value
    assert abs(obs.value - value) < 0.08, 'value must be reported as measured, not clamped'


@pytest.mark.parametrize('value', [0.0, 10.0])
def test_f2_scale_endpoints_are_valid(reader, value):
    obs = observe(reader, gauge(value))
    assert 'OUT_OF_SCALE' not in obs.flags, obs.detail
    assert abs(obs.value - value) < 0.06


def _frame(value, conf=0.9):
    return gr.FrameReading(tag_found=True, value=value, needle_confidence=conf,
                           dial_visibility=1.0, view_angle_deg=10.0, dial_radius_px=60.0,
                           dial_radius_minor_px=60.0, sharpness=500.0, center_refined=True,
                           needle_support=1.0)


@pytest.mark.parametrize('bad', [float('nan'), float('inf'), -float('inf')])
def test_f2_non_finite_frame_values_never_reach_the_estimate(bad):
    obs = gr.fuse([_frame(4.2), _frame(bad), _frame(4.21), _frame(4.19)], 4)
    assert math.isfinite(obs.value) and math.isfinite(obs.sigma) and obs.sigma > 0
    assert obs.frames_valid == 3


def test_f2_all_frames_non_finite_gives_no_value():
    obs = gr.fuse([_frame(float('nan')), _frame(float('inf'))], 2)
    assert not obs.has_value and obs.flags


def test_f2_sigma_is_always_finite_positive_when_value_reported(reader):
    for v in (0.0, 5.0, 10.0):
        obs = observe(reader, gauge(v), n=2)
        if obs.has_value:
            assert math.isfinite(obs.sigma) and obs.sigma > 0
