"""V3 (Astra V2 review, F1 residual): merged-distractor false acceptance.

Case #183 of the seed-20261108 evaluation (tools/heldout_uncertainty_eval.py): a very dark
(24/255), thin (3 px texture) radial line 5.9 deg from the needle, 0.18-0.66 R, blur 5 px.
V2 accepted 1.95804 bar (truth 2.07644, error 0.1184) with no flag: on every sampling ring the
two lines merge into ONE lobe that passes the ambiguity, support and model-residual tests.
On the outer rings the merged lobe is wider than any single needle can be (outer lobe ratio
1.03-1.10 vs <= 0.90 for correct readings on the development set) -> AMBIGUOUS_NEEDLE.

The frames are stored as fixtures (regenerating them needs the generator's RNG state after
183 earlier cases); the parametric tests below are an independent reconstruction.
"""
import json
import os

import cv2
import pytest

from inspectloop_vision import gauge_reader as gr
import test_astra_regressions as T   # noqa: I100 - shared synthetic rendering helpers

HERE = os.path.dirname(os.path.abspath(__file__))
CASE = os.path.join(HERE, 'data', 'astra_v3_case183')


@pytest.fixture(scope='module')
def reader():
    return gr.GaugeReader()


def test_case183_fixture_is_the_reviewed_case():
    p = json.load(open(os.path.join(CASE, 'case183_params.json')))
    assert p['seed'] == 20261108 and p['i'] == 183 and p['subset'] == 'radial_distractor'
    assert abs(p['truth'] - 2.07644) < 1e-5


def test_case183_is_no_longer_accepted_silently(reader):
    p = json.load(open(os.path.join(CASE, 'case183_params.json')))
    frames = [reader.read_frame(cv2.imread(os.path.join(CASE, f'case183_frame{k}.png')), 11)
              for k in range(5)]
    obs = gr.fuse(frames, 5)
    assert 'AMBIGUOUS_NEEDLE' in obs.flags, (obs.flags, obs.detail)
    assert obs.needle_outer_lobe_ratio > gr.MAX_OUTER_LOBE_RATIO
    # the measured value is still reported (not hidden), it just is not trusted
    assert obs.has_value and abs(obs.value - p['truth']) > 0.1


def _obs(reader, tex, blur, scale, n=5):
    fs = []
    for s in range(n):
        img = T.render(tex, scale=scale, noise=2.0, seed=s)
        if blur:
            img = cv2.GaussianBlur(img, (blur, blur), 0)
        fs.append(reader.read_frame(img, 11))
    return gr.fuse(fs, n)


@pytest.mark.parametrize('truth', [2.08, 4.2, 7.0])
@pytest.mark.parametrize('offset', [-8.0, -6.0, -5.0, -4.0, 4.0, 5.0, 6.0, 8.0])
@pytest.mark.parametrize('blur,scale', [(5, 0.36), (5, 0.30), (3, 0.36), (3, 0.50)])
def test_thin_dark_line_merging_with_needle_is_never_silently_wrong(reader, truth, offset, blur,
                                                                     scale):
    tex = T.gauge(truth)
    T.draw_radial(tex, gr.value_to_angle(truth) + offset, r0=0.18, r1=0.66, width=3,
                  color=(24, 24, 24))
    T.assert_not_silently_wrong(_obs(reader, tex, blur, scale), truth)


@pytest.mark.parametrize('truth', [2.08, 4.2, 7.0])
@pytest.mark.parametrize('blur,scale', [(5, 0.36), (5, 0.30), (3, 0.36), (3, 0.50)])
def test_clean_needle_under_the_same_conditions_is_still_accepted(reader, truth, blur, scale):
    """The new bound must not reject the clean case (no blanket rejection of blurred views)."""
    obs = _obs(reader, T.gauge(truth), blur, scale)
    assert obs.has_value and not obs.flags, obs.flags
    assert abs(obs.value - truth) <= 0.03
    assert obs.needle_outer_lobe_ratio <= gr.MAX_OUTER_LOBE_RATIO
