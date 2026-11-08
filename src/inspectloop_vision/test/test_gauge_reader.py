"""Unit tests of the uncertainty-aware gauge reader.

Synthetic scenes warp the *actual generated gauge textures* (the same images Gazebo renders)
into a 640x480 frame with perspective, noise, blur and occluders, so the ground-truth value
is known exactly.
"""
import math
import os

import cv2
import numpy as np
import pytest

from inspectloop_vision import gauge_reader as gr

HERE = os.path.dirname(os.path.abspath(__file__))
SRC_MODELS = os.path.join(HERE, '..', '..', 'inspectloop_gazebo', 'models')
TRUTH = {11: 4.2, 12: 6.8, 13: 3.1, 14: 8.7}


def _models_dir():
    if os.path.isdir(SRC_MODELS):
        return SRC_MODELS
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('inspectloop_gazebo'), 'models')


def texture(tag):
    path = os.path.join(_models_dir(), f'gauge_panel_{tag}', 'materials', 'textures',
                        f'gauge_{tag}.png')
    img = cv2.imread(path)
    assert img is not None, path
    return img


def render(tag, scale=0.32, skew=0.0, noise=2.0, gain=1.0, blur=0, occluder=None, seed=0,
           shift=(0, 0)):
    """Project a gauge texture into a 640x480 scene with a perspective homography."""
    tex = texture(tag)
    scene = np.full((480, 640, 3), (110, 120, 130), np.uint8)
    h, w = tex.shape[:2]
    cx, cy = 320 + shift[0], 245 + shift[1]
    pw, ph = w * scale, h * scale
    k = skew * ph
    dst = np.float32([[cx - pw / 2, cy - ph / 2 - k / 2], [cx + pw / 2, cy - ph / 2 + k / 2],
                      [cx + pw / 2, cy + ph / 2 - k / 2], [cx - pw / 2, cy + ph / 2 + k / 2]])
    H = cv2.getPerspectiveTransform(np.float32([[0, 0], [w, 0], [w, h], [0, h]]), dst)
    m = cv2.warpPerspective(np.full((h, w), 255, np.uint8), H, (640, 480))
    scene[m > 0] = cv2.warpPerspective(tex, H, (640, 480))[m > 0]
    if occluder is not None:   # (x0, y0, x1, y1) in panel-texture pixels, painted dark grey
        x0, y0, x1, y1 = occluder
        poly = cv2.perspectiveTransform(np.float32([[[x0, y0]], [[x1, y0]], [[x1, y1]],
                                                    [[x0, y1]]]), H).astype(np.int32)
        cv2.fillConvexPoly(scene, poly, (78, 80, 84))
    out = scene.astype(np.float32) * gain
    out += np.random.default_rng(seed).normal(0, noise, out.shape)
    out = np.clip(out, 0, 255).astype(np.uint8)
    if blur:
        out = cv2.GaussianBlur(out, (blur, blur), 0)
    return out


@pytest.fixture(scope='module')
def reader():
    return gr.GaugeReader()


def burst(reader, tag, n=5, **kw):
    frames = [reader.read_frame(render(tag, seed=s, **kw), tag) for s in range(n)]
    return gr.fuse(frames, n)


@pytest.mark.parametrize('tag', sorted(TRUTH))
def test_texture_reads_exactly(reader, tag):
    tex = texture(tag)
    big = np.full((900, 900, 3), 120, np.uint8)
    big[100:780, 200:700] = tex
    fr = reader.read_frame(big, tag)
    assert fr.center_refined
    assert abs(fr.value - TRUTH[tag]) < 0.02


@pytest.mark.parametrize('tag', sorted(TRUTH))
@pytest.mark.parametrize('scale,skew', [(0.32, 0.0), (0.32, 0.2), (0.5, -0.25), (0.75, 0.2)])
def test_reading_accuracy_and_clean_flags(reader, tag, scale, skew):
    obs = burst(reader, tag, scale=scale, skew=skew)
    assert obs.has_value and obs.flags == []
    assert abs(obs.value - TRUTH[tag]) < 0.08, obs.detail
    assert abs(obs.value - TRUTH[tag]) <= 3 * obs.sigma


def test_sigma_grows_when_dial_is_small(reader):
    near = burst(reader, 11, scale=0.6)
    far = burst(reader, 11, scale=0.2)
    assert far.sigma > near.sigma
    assert far.dial_radius_px < near.dial_radius_px


def test_occluded_dial_is_flagged(reader):
    # dark box over the upper part of the dial, tag still visible
    obs = burst(reader, 12, occluder=(170, 270, 330, 520))
    assert 'OCCLUDED' in obs.flags
    assert obs.dial_visibility < 0.85


def test_tag_hidden_is_tag_not_visible(reader):
    obs = burst(reader, 13, occluder=(120, 0, 380, 280))
    assert obs.flags == ['TAG_NOT_VISIBLE']
    assert not obs.has_value


def test_dial_outside_image_is_not_accepted(reader):
    obs = burst(reader, 14, scale=0.6, shift=(0, 260))     # dial largely below the frame
    assert obs.flags, obs.detail


def test_no_frames_is_no_image():
    obs = gr.fuse([], 5)
    assert obs.flags == ['NO_IMAGE'] and not obs.has_value


def test_intermittent_tag_and_inconsistency_flags():
    good = [gr.FrameReading(tag_found=True, value=v, needle_confidence=0.9, dial_visibility=1.0,
                            view_angle_deg=10.0, dial_radius_px=60.0, sharpness=500.0)
            for v in (4.0, 4.6, 3.6)]
    obs = gr.fuse(good + [gr.FrameReading()], 4)
    assert 'TAG_INTERMITTENT' in obs.flags
    assert 'INCONSISTENT_FRAMES' in obs.flags


def test_blurred_frames_lower_sharpness(reader):
    sharp = burst(reader, 11)
    blurry = burst(reader, 11, blur=7)
    assert blurry.sharpness < sharp.sharpness


def test_wrong_tag_not_read(reader):
    fr = reader.read_frame(render(11), 12)
    assert not fr.tag_found and fr.detected_ids == [11]


def test_constants_match_textures():
    tex = texture(11)
    assert tex.shape[:2] == (gr.TEX_H, gr.TEX_W)
    x0, y0, x1, y1 = gr.MARKER_BOX
    assert tex[y0 + 2:y0 + 25, x0 + 2:x1 - 2].mean() < 15            # black tag border
    q = gr.QUIET
    assert tex[y0 - q + 2:y0 - 2, x0:x1].mean() > 250                 # white quiet zone
    cx, cy = (int(v) for v in gr.DIAL_CENTER)
    assert tex[cy, cx].mean() < 40                                    # needle hub
    edge = tex[cy, int(cx - gr.DIAL_R) + 3]
    assert edge.mean() < 80                                           # bezel at radius R
    assert abs(gr.TAG_SIZE_M - 0.144) < 1e-9


def test_angle_value_mapping_roundtrip():
    for v in (0.0, 2.5, 5.0, 10.0):
        assert math.isclose(gr.angle_to_value(gr.value_to_angle(v)), v, abs_tol=1e-9)
    assert gr.value_to_angle(5.0) == 0.0


def test_heuristic_sigma_bounds_errors_on_synthetic_views(reader):
    """Consistency check of the HEURISTIC sigma against ground truth (not a calibration).

    Over a fixed grid of scales, perspective skews, noise and blur (the same conditions used to
    develop the model, so this is not held-out evidence - see tools/heldout_uncertainty_eval.py),
    every unflagged observation must have |error| <= 2 sigma in at least 90 % of the cases and
    never > 4 sigma. No probability/coverage claim follows from this test.
    """
    inside, total, worst = 0, 0, 0.0
    for tag, truth in TRUTH.items():
        for scale in (0.24, 0.32, 0.5, 0.75):
            for skew in (-0.25, 0.0, 0.2):
                for blur in (0, 3):
                    obs = burst(reader, tag, n=3, scale=scale, skew=skew, blur=blur, noise=3.0)
                    if obs.flags or not obs.has_value:
                        continue
                    total += 1
                    z = abs(obs.value - truth) / obs.sigma
                    inside += z <= 2.0
                    worst = max(worst, z)
    assert total >= 60
    assert inside / total >= 0.90, f'{inside}/{total} within 2 sigma'
    assert worst <= 4.0, f'worst normalised error {worst:.2f}'


# ---------------------------------------------------------------- real Gazebo frames
DATA = os.path.join(HERE, 'data')


def _gz(name):
    img = cv2.imread(os.path.join(DATA, name))
    assert img is not None, name
    return img


def test_gazebo_frame_oblique_view_with_bracket_over_needle_is_not_accepted(reader):
    """Real frame from the -45 deg viewpoint of PG-12 (truth 6.8). The conduit box's dark
    bracket arm crosses the dial next to the needle and hides its outer half. The v1 reader
    accepted 6.79 here by luck; since the Astra F1 fix such evidence is rejected."""
    fr = reader.read_frame(_gz('gz_pg12_oblique45_truth6.8.png'), 12)
    obs = gr.fuse([fr] * 3, 3)
    assert obs.flags, obs.detail
    assert 35.0 < fr.view_angle_deg < 55.0          # robot stood at the -45 deg viewpoint


def test_gazebo_frame_alarm_range_gauge(reader):
    obs = gr.fuse([reader.read_frame(_gz('gz_pg14_frontal_truth8.7.png'), 14)] * 3, 3)
    assert obs.flags == []
    assert abs(obs.value - 8.7) < 0.08


def test_gazebo_frame_occluded_dial_is_rejected(reader):
    obs = gr.fuse([reader.read_frame(_gz('gz_pg12_frontal_occluded.png'), 12)] * 3, 3)
    assert obs.frames_tag_detected == 3                # tag visible above the conduit box
    assert 'OCCLUDED' in obs.flags


def test_gazebo_frame_blocked_by_crate(reader):
    obs = gr.fuse([reader.read_frame(_gz('gz_pg13_crate_blocked.png'), 13)], 1)
    assert obs.flags == ['TAG_NOT_VISIBLE'] and not obs.has_value
