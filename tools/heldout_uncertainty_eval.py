#!/usr/bin/env python3
"""Held-out evaluation of the gauge reader + decision policy (Astra F6).

Independent of the data used to choose thresholds (fixed values 0.4..9.6, fixed scales, no
in-plane rotation, no gamma): this script draws NEW random conditions from a fixed seed:

  value     U(0, 10) (+ an out-of-scale subset in [-0.6, -0.06] u [10.06, 10.6])
  scale     U(0.20, 0.80)    skew U(-0.30, 0.30)    in-plane roll U(-8, 8) deg
  gain      U(0.5, 1.3)      gamma U(0.7, 1.4)      noise sigma U(1, 6) grey levels
  blur      {0, 3, 5} px Gaussian
  subset    clean 55 % | radial distractor 15 % | non-radial line 10 % | white occluder 10 %
            | shadow wedge 10 %

Each sample is a burst of 5 frames (independent noise) -> gauge_reader.fuse -> the mission's
decision_policy (ACCEPT only if the full contract holds). Reported with denominators:
  yield                  accepted / samples
  incorrect acceptance   accepted with |error| > 0.10 bar  / accepted   (and / samples)
  |error| of accepted    mean, median, 95th percentile, max
  coverage               |error| <= 1 and 2 heuristic sigma / accepted
Rates carry 95 % Wilson score intervals *for the observed proportion on this sample* - that is a
statement about this evaluation, not a calibration of sigma.

Usage: tools/heldout_uncertainty_eval.py [--n 400] [--seed 20261108] [--out file.json]
"""
import argparse
import importlib.util
import json
import math
import os
import sys
import time

import cv2
import numpy as np

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, 'src', 'inspectloop_vision'))
sys.path.insert(0, os.path.join(ROOT, 'src', 'inspectloop_mission'))
from inspectloop_mission import decision_policy as dp  # noqa: E402
from inspectloop_vision import gauge_reader as gr  # noqa: E402

spec = importlib.util.spec_from_file_location(
    'gen', os.path.join(ROOT, 'src', 'inspectloop_gazebo', 'scripts', 'generate_warehouse.py'))
GEN = importlib.util.module_from_spec(spec)
spec.loader.exec_module(GEN)

INCORRECT_TOL = 0.10


def polar(r, a):
    a = math.radians(a)
    return (gr.DIAL_CENTER[0] + r * gr.DIAL_R * math.sin(a),
            gr.DIAL_CENTER[1] - r * gr.DIAL_R * math.cos(a))


def make_texture(rng, value, subset):
    tex = GEN.make_gauge_texture(11, 'PG-11', value)
    na = gr.value_to_angle(value)
    if subset == 'radial_distractor':
        off = rng.choice([-1, 1]) * rng.uniform(3, 40)
        dark = int(rng.uniform(10, 90))
        p0 = tuple(int(v) for v in polar(rng.uniform(0.1, 0.3), na + off))
        p1 = tuple(int(v) for v in polar(rng.uniform(0.55, 0.85), na + off))
        cv2.line(tex, p0, p1, (dark,) * 3, int(rng.integers(3, 9)), cv2.LINE_AA)
    elif subset == 'line_across':
        a0, a1 = na + rng.uniform(-60, -10), na + rng.uniform(10, 60)
        p0 = tuple(int(v) for v in polar(rng.uniform(0.4, 0.9), a0))
        p1 = tuple(int(v) for v in polar(rng.uniform(0.4, 0.9), a1))
        cv2.line(tex, p0, p1, (int(rng.uniform(10, 80)),) * 3, int(rng.integers(3, 9)), cv2.LINE_AA)
    elif subset == 'white_occluder':
        c = na + rng.uniform(-20, 20)
        w = rng.uniform(8, 30)
        r0 = rng.uniform(0.1, 0.5)
        pts = [polar(r0, a) for a in np.linspace(c - w, c + w, 15)] + \
              [polar(r0 + rng.uniform(0.2, 0.6), a) for a in np.linspace(c + w, c - w, 15)]
        cv2.fillPoly(tex, [np.array(pts, np.int32)], (248, 248, 248), cv2.LINE_AA)
    elif subset == 'shadow':
        c = na + rng.choice([-1, 1]) * rng.uniform(4, 25)
        w = rng.uniform(3, 10)
        g = int(rng.uniform(50, 160))
        pts = [polar(0.12, a) for a in np.linspace(c - w, c + w, 10)] + \
              [polar(0.7, a) for a in np.linspace(c + w, c - w, 10)]
        cv2.fillPoly(tex, [np.array(pts, np.int32)], (g, g, g), cv2.LINE_AA)
    return tex


def render(tex, rng, cond, seed):
    scene = np.full((480, 640, 3), (110, 120, 130), np.uint8)
    h, w = tex.shape[:2]
    cx, cy = 320 + rng.uniform(-60, 60), 245 + rng.uniform(-30, 30)
    pw, ph = w * cond['scale'], h * cond['scale']
    k = cond['skew'] * ph
    dst = np.float32([[-pw / 2, -ph / 2 - k / 2], [pw / 2, -ph / 2 + k / 2],
                      [pw / 2, ph / 2 - k / 2], [-pw / 2, ph / 2 + k / 2]])
    rr = math.radians(cond['roll'])
    R = np.array([[math.cos(rr), -math.sin(rr)], [math.sin(rr), math.cos(rr)]], np.float32)
    dst = (dst @ R.T + np.float32([cx, cy])).astype(np.float32)
    H = cv2.getPerspectiveTransform(np.float32([[0, 0], [w, 0], [w, h], [0, h]]), dst)
    m = cv2.warpPerspective(np.full((h, w), 255, np.uint8), H, (640, 480))
    scene[m > 0] = cv2.warpPerspective(tex, H, (640, 480))[m > 0]
    img = (scene.astype(np.float32) / 255.0) ** cond['gamma'] * 255.0 * cond['gain']
    img += np.random.default_rng(seed).normal(0, cond['noise'], img.shape)
    img = np.clip(img, 0, 255).astype(np.uint8)
    if cond['blur']:
        img = cv2.GaussianBlur(img, (cond['blur'], cond['blur']), 0)
    return img


def wilson(k, n, z=1.96):
    if n == 0:
        return [None, None]
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return [round(max(0.0, c - h), 4), round(min(1.0, c + h), 4)]


def rate(k, n):
    return {'k': k, 'n': n, 'rate': round(k / n, 4) if n else None, 'wilson95': wilson(k, n)}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--n', type=int, default=400)
    ap.add_argument('--seed', type=int, default=20261108)
    ap.add_argument('--out', default=None)
    a = ap.parse_args(argv)
    rng = np.random.default_rng(a.seed)
    reader = gr.GaugeReader()
    policy = dp.PolicyParams()
    subsets = ['clean', 'radial_distractor', 'line_across', 'white_occluder', 'shadow']
    probs = [0.55, 0.15, 0.10, 0.10, 0.10]
    rows = []
    t0 = time.time()
    for i in range(a.n):
        subset = str(rng.choice(subsets, p=probs))
        oos = rng.random() < 0.08
        value = (float(rng.choice([rng.uniform(-0.6, -0.06), rng.uniform(10.06, 10.6)]))
                 if oos else float(rng.uniform(0.0, 10.0)))
        cond = {'scale': float(rng.uniform(0.2, 0.8)), 'skew': float(rng.uniform(-0.3, 0.3)),
                'roll': float(rng.uniform(-8, 8)), 'gain': float(rng.uniform(0.5, 1.3)),
                'gamma': float(rng.uniform(0.7, 1.4)), 'noise': float(rng.uniform(1, 6)),
                'blur': int(rng.choice([0, 3, 5]))}
        tex = make_texture(rng, value, subset)
        frames = [reader.read_frame(render(tex, rng, cond, a.seed * 1000 + i * 10 + s), 11)
                  for s in range(5)]
        obs = gr.fuse(frames, 5)
        o = {'has_value': obs.has_value, 'value': obs.value if obs.has_value else None,
             'sigma': obs.sigma if obs.has_value else None, 'frames_valid': obs.frames_valid,
             'frame_spread': obs.frame_spread, 'quality_flags': list(obs.flags)}
        d = dp.decide(o, 0, 2, 1, policy)
        err = (obs.value - value) if obs.has_value else None
        rows.append({'i': i, 'subset': subset, 'out_of_scale_truth': oos, 'truth': value,
                     'cond': cond, 'value': o['value'], 'sigma': o['sigma'],
                     'error': err, 'flags': o['quality_flags'], 'decision': d.decision,
                     'dial_radius_minor_px': obs.dial_radius_minor_px})
    acc = [r for r in rows if r['decision'] == dp.ACCEPT]
    errs = np.array([abs(r['error']) for r in acc]) if acc else np.array([])

    def block(rs):
        ac = [r for r in rs if r['decision'] == dp.ACCEPT]
        e = [abs(r['error']) for r in ac]
        return {'samples': len(rs), 'accepted': len(ac), 'yield': rate(len(ac), len(rs)),
                'incorrect_acceptance_of_accepted': rate(sum(x > INCORRECT_TOL for x in e), len(ac)),
                'incorrect_acceptance_of_samples': rate(sum(x > INCORRECT_TOL for x in e), len(rs)),
                'mean_abs_error': round(float(np.mean(e)), 4) if e else None,
                'max_abs_error': round(float(np.max(e)), 4) if e else None,
                'within_1sigma': rate(sum(abs(r['error']) <= r['sigma'] for r in ac), len(ac)),
                'within_2sigma': rate(sum(abs(r['error']) <= 2 * r['sigma'] for r in ac), len(ac))}

    res = {
        'seed': a.seed, 'n': a.n, 'runtime_s': round(time.time() - t0, 1),
        'incorrect_tolerance_bar': INCORRECT_TOL,
        'note': 'synthetic renders of the generated gauge texture; sigma is heuristic; Wilson '
                'intervals describe the observed proportions on this sample only',
        'overall': block(rows),
        'accepted_abs_error': {
            'mean': round(float(errs.mean()), 4) if len(errs) else None,
            'median': round(float(np.median(errs)), 4) if len(errs) else None,
            'p95': round(float(np.percentile(errs, 95)), 4) if len(errs) else None,
            'max': round(float(errs.max()), 4) if len(errs) else None},
        'by_subset': {s: block([r for r in rows if r['subset'] == s]) for s in subsets},
        'out_of_scale_truth': block([r for r in rows if r['out_of_scale_truth']]),
        'incorrect_acceptances': [r for r in acc if abs(r['error']) > INCORRECT_TOL],
    }
    if a.out:
        with open(a.out, 'w') as f:
            json.dump({'summary': res, 'rows': rows}, f, indent=1)
    o = res['overall']
    print(f"samples {o['samples']}  accepted {o['accepted']}  yield {o['yield']['rate']} "
          f"{o['yield']['wilson95']}")
    print(f"incorrect acceptance (|err|>{INCORRECT_TOL}): {o['incorrect_acceptance_of_accepted']['k']}"
          f"/{o['accepted']} accepted {o['incorrect_acceptance_of_accepted']['wilson95']}")
    print(f"accepted |err|: {res['accepted_abs_error']}")
    print(f"within 1 sigma_h {o['within_1sigma']['k']}/{o['accepted']}, within 2 sigma_h "
          f"{o['within_2sigma']['k']}/{o['accepted']}")
    for s, b in res['by_subset'].items():
        print(f"  {s:<18} n={b['samples']:<4} accepted={b['accepted']:<4} incorrect="
              f"{b['incorrect_acceptance_of_accepted']['k']} max|err|={b['max_abs_error']}")
    b = res['out_of_scale_truth']
    print(f"  out-of-scale truth n={b['samples']} accepted={b['accepted']}")
    return 0


if __name__ == '__main__':
    sys.exit(main())
