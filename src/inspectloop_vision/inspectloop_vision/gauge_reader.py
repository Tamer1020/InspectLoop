"""Analog gauge reading with explicit uncertainty (ROS independent, unit tested).

Per frame
---------
1. Detect the ArUco tag (DICT_4X4_50) that identifies the gauge panel.
2. Homography  H : panel-texture pixels -> image pixels  from the 4 tag corners
   (the tag and dial are printed on the same rigid plane).
3. Rectify the image into the canonical, fronto-parallel panel frame (500 x 600 px).
   Pixels that fall outside the camera image are tracked with a validity mask.
4. Needle (Astra F1): on each of 16 sampling rings (0.15-0.52 R) the needle must appear as ONE
   dark lobe of the width predicted by the needle geometry, and the rings must agree on its
   angle (support). A competing lobe anywhere on a ring (ambiguity) or dark structure near the
   needle that a fitted tapered-needle model cannot explain (model residual) makes the reading
   AMBIGUOUS_NEEDLE; rings that do not see the needle make it NEEDLE_PARTIAL. The search sweep
   extends 20 deg beyond the scale so out-of-scale needles are measured and flagged, not clamped.
5. Quality evidence
   * needle_confidence: contrast x support x (1 - ambiguity);
   * dial_visibility: fraction of the dial face annulus that looks like the white face
     (bright, unsaturated, inside the image) - occluders lower it;
   * view angle / distance from solvePnP on the tag (needs camera intrinsics);
   * apparent dial radius in image pixels (resolution) and Laplacian sharpness.

Per observation (a burst of frames from one viewpoint)
------------------------------------------------------
`fuse()` takes the median of the valid frame readings and estimates a 1-sigma uncertainty

    sigma^2 = (1.25 * spread / sqrt(n))^2 + sigma_res^2 + sigma_floor^2

  spread      robust frame-to-frame spread (1.4826 * MAD)  -> random error
  sigma_res   value change for a 0.5 px needle displacement at 0.35 R measured along the
              foreshortened (minor) axis of the projected dial -> resolution / obliqueness
  sigma_floor calibration floor (0.03 bar)

and raises quality flags (OCCLUDED, OBLIQUE_VIEW, TOO_FAR, AMBIGUOUS_NEEDLE, NEEDLE_PARTIAL, ...).
The flags and sigma feed the mission's decision policy (ACCEPT / RETRY_SAME / MOVE_NEW /
UNRESOLVED). sigma is a HEURISTIC: it is not a calibrated probability and value +/- k*sigma is not
a confidence interval (Astra F6). docs/ASTRA_FIX_REPORT.md reports held-out error statistics.

Texture constants mirror inspectloop_gazebo/scripts/generate_warehouse.py (checked by tests).
"""
from dataclasses import dataclass, field
import math
import time
import warnings
from typing import Dict, List, Optional, Sequence

import cv2
import numpy as np

# ---------------------------------------------------------------- panel geometry
TEX_W, TEX_H = 500, 680
MARKER_BOX = (160, 40, 340, 220)
QUIET = 30
DIAL_CENTER = (250.0, 465.0)
DIAL_R = 190.0
PX_TO_M = 0.0008                      # 0.40 m x 0.544 m panel / 500 x 680 px
TAG_SIZE_M = (MARKER_BOX[2] - MARKER_BOX[0]) * PX_TO_M
ANGLE_MIN_DEG, ANGLE_MAX_DEG = -135.0, 135.0
VALUE_MIN, VALUE_MAX = 0.0, 10.0
UNIT = 'bar'
VALUE_PER_DEG = (VALUE_MAX - VALUE_MIN) / (ANGLE_MAX_DEG - ANGLE_MIN_DEG)

# ---------------------------------------------------------------- algorithm constants
RAY_R0, RAY_R1, RAY_SAMPLES = 0.15, 0.52, 16
# The needle search covers the scale plus a margin, so a needle *beyond* the scale is still
# measured (and flagged OUT_OF_SCALE) instead of being forced onto the nearest scale end.
SWEEP_MARGIN_DEG = 20.0
ANGLE_STEP_DEG = 0.5
MIN_RAY_VALID_FRACTION = 0.6
# Needle geometry (generator): tapered polygon, half-width 0.035 R at the pivot, tip at 0.82 R.
NEEDLE_BASE_HALFWIDTH = 0.035
NEEDLE_TIP = 0.82
# Per-ring needle analysis (Astra F1)
RING_MIN_CONTRAST = 0.15          # darkness above the ring median for a ring to 'see' a line
OUTER_RING_R = 0.28               # rings beyond this resolve lines a few degrees apart
AMBIGUITY_LIMIT = 0.45            # competing line >= 45 % of the needle's contrast -> ambiguous
# Needle-model residual: energy of the dark structure within +-20 deg of the needle that the
# fitted (tapered, blurred) needle model does NOT explain, relative to the model's energy.
# Measured on synthetic renders with a minor dial radius >= 40 px: clean needles <= 0.071,
# any nearby distractor that moved the reading by > 0.1 bar >= 0.064 (only 3 deg offsets below
# 0.075). Below 40 px clean and distractor residuals overlap -> TOO_FAR (min_dial_radius_px).
RESIDUAL_WINDOW_DEG = 20.0
MAX_MODEL_RESIDUAL = 0.075
MIN_NEEDLE_SUPPORT = 0.70         # fraction of rings that see the needle at the consensus angle
OUT_OF_SCALE_TOL = 0.05           # bar beyond the printed scale before flagging OUT_OF_SCALE
VIS_R0, VIS_R1 = 0.15, 0.52
VIS_EXCLUDE = [(150.0, 210.0)]        # 'bar' caption below the hub (degrees, clockwise from up)
FRAME_MIN_CONFIDENCE = 0.20           # below: the frame does not produce a reading
# Needle localisation accuracy in image pixels (perpendicular to the needle) used by the
# resolution term of sigma. Averaging 16 ray samples per angle gives sub-pixel localisation;
# the value was chosen on synthetic renders (test_heuristic_sigma_bounds_errors_on_synthetic_views);
# it is a heuristic, not a statistically calibrated quantity.
NEEDLE_LOCALISATION_PX = 0.5

FLAGS = ('NO_IMAGE', 'TAG_NOT_VISIBLE', 'TAG_INTERMITTENT', 'OCCLUDED', 'OBLIQUE_VIEW', 'TOO_FAR',
         'LOW_NEEDLE_CONTRAST', 'AMBIGUOUS_NEEDLE', 'NEEDLE_PARTIAL', 'INCONSISTENT_FRAMES',
         'BLURRED', 'OUT_OF_SCALE')


_erf = np.vectorize(math.erf, otypes=[float])


def needle_halfwidth_deg(r_frac):
    """Angular half-width of the printed needle at radius r (fraction of R)."""
    r = np.maximum(np.asarray(r_frac, float), 1e-3)
    hw = NEEDLE_BASE_HALFWIDTH * np.clip(1.0 - r / NEEDLE_TIP, 0.0, 1.0)
    return np.degrees(np.arctan2(hw, r))


@dataclass
class FuseParams:
    min_valid_frames: int = 3
    min_visibility: float = 0.85
    max_view_angle_deg: float = 55.0
    min_dial_radius_px: float = 40.0     # below this the needle-model test lacks evidence (F1)
    min_needle_confidence: float = 0.35
    max_frame_spread: float = 0.20
    min_sharpness: float = 15.0
    sigma_floor: float = 0.03


@dataclass
class FrameReading:
    tag_found: bool = False
    value: Optional[float] = None
    angle_deg: Optional[float] = None
    needle_confidence: float = 0.0
    dial_visibility: float = 0.0
    view_angle_deg: Optional[float] = None
    distance_m: Optional[float] = None
    dial_radius_px: float = 0.0
    dial_radius_minor_px: float = 0.0
    sharpness: float = 0.0
    tag_corners: Optional[np.ndarray] = None
    homography: Optional[np.ndarray] = None
    detected_ids: List[int] = field(default_factory=list)
    profile: Optional[np.ndarray] = None
    dial_center_tex: Optional[tuple] = None
    center_refined: bool = False
    needle_support: float = 0.0       # fraction of rings seeing the needle at the consensus angle
    needle_ambiguity: float = 1.0     # strength of the best competing line / needle (0 = none)
    needle_lobe_ratio: float = 0.0    # measured / predicted needle lobe width (diagnostic)
    needle_residual: float = 1.0      # unexplained dark energy near the needle / needle energy
    out_of_range: bool = False        # needle at the edge of the search sweep (beyond the scale)
    detail: str = ''

    @property
    def valid(self) -> bool:
        return (self.value is not None and math.isfinite(self.value)
                and self.needle_confidence >= FRAME_MIN_CONFIDENCE)


@dataclass
class Observation:
    has_value: bool
    value: float
    sigma: float
    flags: List[str]
    frames_requested: int
    frames_received: int
    frames_tag_detected: int
    frames_valid: int
    frame_values: List[float]
    frame_spread: float
    needle_confidence: float
    dial_visibility: float
    view_angle_deg: float
    distance_m: float
    dial_radius_px: float
    sharpness: float
    detail: str
    dial_radius_minor_px: float = 0.0
    needle_support: float = 0.0
    needle_ambiguity: float = 1.0
    needle_residual: float = 1.0


def value_to_angle(v: float) -> float:
    return ANGLE_MIN_DEG + (v - VALUE_MIN) / VALUE_PER_DEG


def angle_to_value(a: float) -> float:
    return VALUE_MIN + (a - ANGLE_MIN_DEG) * VALUE_PER_DEG


def default_camera_matrix(width=640, height=480, hfov=1.047) -> np.ndarray:
    f = (width / 2.0) / math.tan(hfov / 2.0)
    return np.array([[f, 0, width / 2.0], [0, f, height / 2.0], [0, 0, 1]], np.float64)


def _make_detector():
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    if hasattr(cv2.aruco, 'ArucoDetector'):            # OpenCV >= 4.7
        params = cv2.aruco.DetectorParameters()
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        det = cv2.aruco.ArucoDetector(dictionary, params)
        return lambda grey: det.detectMarkers(grey)
    params = cv2.aruco.DetectorParameters_create()      # OpenCV 4.6 (Ubuntu 24.04)
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return lambda grey: cv2.aruco.detectMarkers(grey, dictionary, parameters=params)


def _polar_xy(r_frac: np.ndarray, ang_deg: np.ndarray):
    a = np.radians(ang_deg)
    x = DIAL_CENTER[0] + r_frac * DIAL_R * np.sin(a)
    y = DIAL_CENTER[1] - r_frac * DIAL_R * np.cos(a)
    return x, y


def _fit_circle(pts: np.ndarray):
    """Algebraic least-squares circle fit. Returns (cx, cy, r)."""
    x, y = pts[:, 0], pts[:, 1]
    A = np.column_stack([2 * x, 2 * y, np.ones_like(x)])
    b = x * x + y * y
    (cx, cy, c), *_ = np.linalg.lstsq(A, b, rcond=None)
    return float(cx), float(cy), float(math.sqrt(max(c + cx * cx + cy * cy, 0.0)))


def refine_dial_center(dark: np.ndarray, maskf: np.ndarray):
    """Locate the outer edge of the dark bezel (r = 1.0 R) and fit a circle to it.

    Small errors in the 4 tag corners are magnified when the homography is extrapolated
    to the dial (it lies ~1.6 tag-lengths below the tag); a wrong pivot point biases the
    needle angle. Rays are searched inward from 1.06 R (still on the light panel) to the
    first dark sample; the top +-35 deg are skipped because the tag sits just above the dial.
    Returns (cx, cy, M, n_points): centre and 2x2 stretch (unit circle * R -> fitted ellipse) in
    panel-texture pixels; falls back to the nominal centre and identity.
    """
    angs_deg = np.arange(0.0, 360.0, 4.0)
    angs_deg = angs_deg[np.abs(((angs_deg + 180.0) % 360.0) - 180.0) > 35.0]
    angs = np.radians(angs_deg)
    rs = np.linspace(1.06, 0.90, 33) * DIAL_R
    aa, rr = np.meshgrid(angs, rs, indexing='ij')
    xs = (DIAL_CENTER[0] + rr * np.sin(aa)).astype(np.float32)
    ys = (DIAL_CENTER[1] - rr * np.cos(aa)).astype(np.float32)
    d = cv2.remap(dark, xs, ys, cv2.INTER_LINEAR)
    v = cv2.remap(maskf, xs, ys, cv2.INTER_NEAREST) > 0.5
    pts = []
    for i in range(len(angs)):
        if not v[i].all() or d[i, 0] > 0.35:      # off-image ray, or not on light panel
            continue
        hit = np.nonzero(d[i] > 0.5)[0]
        if len(hit) == 0 or hit[0] == 0 or hit[0] + 2 >= d.shape[1]:
            continue
        j = hit[0]
        if not (d[i, j:j + 3] > 0.5).all():
            continue
        f = (0.5 - d[i, j - 1]) / max(d[i, j] - d[i, j - 1], 1e-6)
        r_edge = rs[j - 1] + f * (rs[j] - rs[j - 1])
        pts.append((DIAL_CENTER[0] + r_edge * math.sin(angs[i]),
                    DIAL_CENTER[1] - r_edge * math.cos(angs[i])))
    if len(pts) < 15:
        return DIAL_CENTER[0], DIAL_CENTER[1], np.eye(2), len(pts)
    pts = np.array(pts)
    cx, cy, r = _fit_circle(pts)
    for _ in range(2):                              # reject outliers (circle model), refit
        res = np.abs(np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - r)
        keep = res < 3.0
        if keep.sum() < 15:
            break
        pts = pts[keep]
        cx, cy, r = _fit_circle(pts)
    # ellipse model: the residual homography error is mostly an anisotropic stretch
    (ex, ey), (w, h), theta = cv2.fitEllipse(pts.astype(np.float32))
    c, s_ = math.cos(math.radians(theta)), math.sin(math.radians(theta))
    rot = np.array([[c, -s_], [s_, c]])
    M = rot @ np.diag([w / (2 * DIAL_R), h / (2 * DIAL_R)]) @ rot.T
    if (math.hypot(ex - DIAL_CENTER[0], ey - DIAL_CENTER[1]) > 0.1 * DIAL_R or
            not 0.85 < min(w, h) / (2 * DIAL_R) <= max(w, h) / (2 * DIAL_R) < 1.15):
        return DIAL_CENTER[0], DIAL_CENTER[1], np.eye(2), 0
    return float(ex), float(ey), M, len(pts)


class GaugeReader:
    def __init__(self):
        self._detect = _make_detector()
        x0, y0, x1, y1 = MARKER_BOX
        self._tag_tex = np.array([[x0, y0], [x1, y0], [x1, y1], [x0, y1]], np.float32)
        s = TAG_SIZE_M / 2.0
        self._tag_obj = np.array([[-s, s, 0], [s, s, 0], [s, -s, 0], [-s, -s, 0]], np.float64)
        # ray sampling grid (angles x radii) in texture coordinates
        lo, hi = ANGLE_MIN_DEG - SWEEP_MARGIN_DEG, ANGLE_MAX_DEG + SWEEP_MARGIN_DEG
        self._angles = np.arange(lo, hi + 1e-9, ANGLE_STEP_DEG)
        radii = np.linspace(RAY_R0, RAY_R1, RAY_SAMPLES)
        self._radii = radii
        self._ray_aa, self._ray_rr = np.meshgrid(self._angles, radii, indexing='ij')
        # visibility annulus samples (full circle, 2 deg x 10 radii)
        va = np.arange(0.0, 360.0, 2.0)
        vr = np.linspace(VIS_R0, VIS_R1, 10)
        vaa, vrr = np.meshgrid(va, vr, indexing='ij')
        self._vis_ang = vaa
        self._vis_r = vrr
        self._vis_x, self._vis_y = (v.astype(np.float32) for v in _polar_xy(vrr, vaa))
        # quiet zone around the tag (white reference)
        self._white_ref_box = (MARKER_BOX[0] - QUIET + 4, MARKER_BOX[1] - QUIET + 4,
                               MARKER_BOX[2] + QUIET - 4, MARKER_BOX[3] + QUIET - 4)

    # ------------------------------------------------------------------ per frame
    def read_frame(self, bgr: np.ndarray, tag_id: int,
                   camera_matrix: Optional[np.ndarray] = None) -> FrameReading:
        fr = FrameReading()
        if bgr is None or bgr.ndim != 3 or bgr.shape[2] != 3 or bgr.size == 0:
            fr.detail = 'invalid image buffer'
            return fr
        grey = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detect(grey)
        found: Dict[int, np.ndarray] = {}
        if ids is not None:
            for c, i in zip(corners, ids.flatten()):
                found[int(i)] = c.reshape(4, 2).astype(np.float32)
        fr.detected_ids = sorted(found)
        if tag_id not in found:
            fr.detail = f'tag {tag_id} not detected (saw {fr.detected_ids})'
            return fr
        fr.tag_found = True
        tc = found[tag_id]
        fr.tag_corners = tc
        H = cv2.getPerspectiveTransform(self._tag_tex, tc)
        fr.homography = H

        # geometry: pose of the tag plane
        K = camera_matrix if camera_matrix is not None else default_camera_matrix(
            bgr.shape[1], bgr.shape[0])
        flag = getattr(cv2, 'SOLVEPNP_IPPE_SQUARE', cv2.SOLVEPNP_ITERATIVE)
        ok, rvec, tvec = cv2.solvePnP(self._tag_obj, tc.astype(np.float64), K, None, flags=flag)
        if ok:
            R, _ = cv2.Rodrigues(rvec)
            n = R[:, 2]
            t = tvec.reshape(3)
            fr.distance_m = float(np.linalg.norm(t))
            cosang = abs(float(np.dot(n, -t))) / max(fr.distance_m, 1e-9)
            fr.view_angle_deg = float(math.degrees(math.acos(min(1.0, cosang))))

        # apparent dial radius in the image
        ring = np.array([_polar_xy(np.array(1.0), np.array(a)) for a in range(0, 360, 30)],
                        np.float32).reshape(-1, 1, 2)
        ctr = np.array([[DIAL_CENTER]], np.float32)
        ring_i = cv2.perspectiveTransform(ring, H).reshape(-1, 2)
        ctr_i = cv2.perspectiveTransform(ctr, H).reshape(2)
        radii_i = np.linalg.norm(ring_i - ctr_i, axis=1)
        fr.dial_radius_px = float(np.mean(radii_i))
        fr.dial_radius_minor_px = float(np.min(radii_i))   # foreshortened (worst) direction

        # sharpness over the dial bounding box (clipped to the image)
        h, w = grey.shape
        x0, y0 = np.clip(ring_i.min(0).astype(int), 0, [w - 1, h - 1])
        x1, y1 = np.clip(ring_i.max(0).astype(int), 0, [w - 1, h - 1])
        if x1 - x0 > 4 and y1 - y0 > 4:
            fr.sharpness = float(cv2.Laplacian(grey[y0:y1, x0:x1], cv2.CV_64F).var())

        # rectification into the panel frame + validity mask
        rect = cv2.warpPerspective(bgr, H, (TEX_W, TEX_H),
                                   flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
        mask = cv2.warpPerspective(np.full(grey.shape, 255, np.uint8), H, (TEX_W, TEX_H),
                                   flags=cv2.INTER_NEAREST | cv2.WARP_INVERSE_MAP)
        rgrey = cv2.cvtColor(rect, cv2.COLOR_BGR2GRAY).astype(np.float32)
        rsat = cv2.cvtColor(rect, cv2.COLOR_BGR2HSV)[:, :, 1].astype(np.float32)
        maskf = mask.astype(np.float32) / 255.0

        bx0, by0, bx1, by1 = self._white_ref_box
        ref_region = rgrey[by0:by1, bx0:bx1]
        ref_mask = np.ones_like(ref_region, bool)
        m = QUIET - 8
        ref_mask[m:-m, m:-m] = False           # keep only the white quiet-zone frame
        white = float(np.percentile(ref_region[ref_mask], 75))
        if white < 30.0:
            fr.detail = 'tag region too dark'
            return fr

        # needle profile around the refined dial centre
        dark = np.clip(1.0 - rgrey / white, 0.0, 1.0)
        cx, cy, M, n_edge = refine_dial_center(dark, maskf)
        fr.dial_center_tex = (cx, cy)
        fr.center_refined = n_edge > 0
        a_rad = np.radians(self._ray_aa)
        ux = self._ray_rr * DIAL_R * np.sin(a_rad)
        uy = -self._ray_rr * DIAL_R * np.cos(a_rad)
        ray_x = (cx + M[0, 0] * ux + M[0, 1] * uy).astype(np.float32)
        ray_y = (cy + M[1, 0] * ux + M[1, 1] * uy).astype(np.float32)
        samples = cv2.remap(dark, ray_x, ray_y, cv2.INTER_LINEAR)
        valid = cv2.remap(maskf, ray_x, ray_y, cv2.INTER_NEAREST) > 0.5
        prof = np.where(valid, samples, np.nan)          # angles x rings
        vfrac = valid.mean(axis=1)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)   # all-NaN rays (off-image) are expected
            score = np.nanmean(prof, axis=1)
        score[vfrac < MIN_RAY_VALID_FRACTION] = np.nan
        fr.profile = score
        if np.all(np.isnan(score)):
            fr.detail = 'dial outside the image'
            fr.dial_visibility = 0.0
            return fr

        # image pixel -> texture pixel scale (blur widens the needle lobe in the texture frame)
        up = DIAL_R / max(fr.dial_radius_minor_px, 1.0)
        res = self._analyse_needle(prof, up)
        if res is None:
            fr.detail = 'no needle-like line found'
            fr.dial_visibility = self._visibility(rgrey, rsat, maskf, white, None, up)
            fr.needle_support, fr.needle_ambiguity = 0.0, 1.0
            return fr
        ang, contrast, support, ambiguity, lobe_ratio = res
        fr.needle_support, fr.needle_ambiguity, fr.needle_lobe_ratio = support, ambiguity, lobe_ratio
        fr.needle_residual = self._model_residual(prof, ang, up)
        conf = min(1.0, contrast / 0.5) * support * max(0.0, 1.0 - ambiguity)
        if not fr.center_refined:
            conf *= 0.5   # pivot not confirmed by the bezel fit -> angle may be biased
        fr.angle_deg = ang
        fr.needle_confidence = float(conf)
        fr.dial_visibility = self._visibility(rgrey, rsat, maskf, white, ang, up)

        edge = SWEEP_MARGIN_DEG - 1.0
        if ang <= ANGLE_MIN_DEG - edge or ang >= ANGLE_MAX_DEG + edge:
            # needle at the end of the search sweep: its true position may lie beyond it
            fr.out_of_range = True
        else:
            fr.value = float(angle_to_value(ang))
        fr.detail = (f'needle {ang:+.1f} deg conf {conf:.2f} support {support:.2f} '
                     f'ambiguity {ambiguity:.2f} residual {fr.needle_residual:.3f} '
                     f'vis {fr.dial_visibility:.2f}'
                     + ('' if fr.center_refined else ' (bezel fit failed)'))
        return fr

    def _analyse_needle(self, prof: np.ndarray, up: float):
        """Per-ring needle evidence (Astra F1).

        A needle is ONE straight, tapered dark line through the pivot. On every sampling ring it
        must appear as a single lobe whose width matches the needle geometry (+ image blur), and
        all rings must agree on its angle. Returns (angle, contrast, support, ambiguity,
        lobe_ratio) or None when no ring sees a line.
          support    fraction of usable rings whose lobe lies at the consensus angle
          ambiguity  75th percentile over outer rings of (best competing lobe / needle lobe)
          lobe_ratio median over outer rings of measured / predicted lobe width
        """
        n_ang, n_ring = prof.shape
        exp_hw = needle_halfwidth_deg(self._radii) + np.degrees(1.5 * up / (self._radii * DIAL_R))
        rings = []      # (j, theta, contrast, ambiguity, lobe_ratio)
        usable = 0
        for j in range(n_ring):
            p = prof[:, j]
            ok = ~np.isnan(p)
            if ok.mean() < 0.6:
                continue
            usable += 1
            med = float(np.median(p[ok]))
            k = int(np.nanargmax(p))
            c = float(p[k]) - med
            if c < RING_MIN_CONTRAST:
                continue                                       # ring does not see a line
            thr = med + 0.5 * c
            lo, hi = k, k
            while lo > 0 and ok[lo - 1] and p[lo - 1] > thr:
                lo -= 1
            while hi < n_ang - 1 and ok[hi + 1] and p[hi + 1] > thr:
                hi += 1
            width = (hi - lo + 1) * ANGLE_STEP_DEG
            excl = exp_hw[j] + 2.0
            outside = ok & ((self._angles < self._angles[lo] - excl) |
                            (self._angles > self._angles[hi] + excl))
            comp = float(np.max(p[outside])) if outside.any() else med
            amb = max(0.0, comp - med) / c
            # darkness-weighted centroid of the lobe (robust to the flat, anti-aliased top of a
            # wide needle lobe, unlike arg-max + parabola)
            wl = np.clip(p[lo:hi + 1] - med, 0.0, None)
            theta = (float(np.sum(wl * self._angles[lo:hi + 1]) / np.sum(wl)) if wl.sum() > 0
                     else float(self._angles[k]))
            rings.append((j, theta, c, amb, width / (2.0 * exp_hw[j])))
        if not rings or usable == 0:
            return None
        outer = [r for r in rings if self._radii[r[0]] >= OUTER_RING_R] or rings
        th = np.array([r[1] for r in outer])
        wt = np.array([r[2] for r in outer])
        order = np.argsort(th)
        cum = np.cumsum(wt[order])
        consensus = float(th[order][np.searchsorted(cum, 0.5 * cum[-1])])   # weighted median
        consistent = [r for r in rings
                      if abs(r[1] - consensus) <= exp_hw[r[0]] + 1.5]
        support = len(consistent) / usable
        cons_outer = [r for r in consistent if self._radii[r[0]] >= OUTER_RING_R] or consistent
        if not cons_outer:
            return None
        w = np.array([r[2] for r in cons_outer])
        angle = float(np.sum(w * np.array([r[1] for r in cons_outer])) / np.sum(w))
        contrast = float(np.mean([r[2] for r in consistent]))
        ambiguity = float(np.percentile([r[3] for r in outer], 75))
        lobe_ratio = float(np.median([r[4] for r in outer]))
        return angle, contrast, support, ambiguity, lobe_ratio

    def _model_residual(self, prof: np.ndarray, angle: float, up: float) -> float:
        """Fit the physical needle model and return the unexplained dark energy (Astra F1).

        Model per ring r: darkness(theta) = base_r + a * [box of the needle's angular half-width
        hw(r)] convolved with a Gaussian blur (sigma = b image px), within +-20 deg of the needle.
        The amplitude a is least-squares fitted, the blur b chosen from a small set. A second line,
        a shadow or a cable next to the needle cannot be explained by one tapered needle and
        leaves residual energy comparable to the needle's own."""
        m_ang = np.abs(self._angles - angle) <= RESIDUAL_WINDOW_DEG
        x = self._angles[m_ang] - angle
        best = 1.0
        for blur in (0.4, 0.7, 1.0, 1.4, 1.8):
            tmpl, obs = [], []
            for j, rad in enumerate(self._radii):
                p = prof[m_ang, j]
                ok = ~np.isnan(p)
                if ok.sum() < 10:
                    continue
                full = prof[:, j]
                base = float(np.median(full[~np.isnan(full)]))
                hw = float(needle_halfwidth_deg(rad))
                sb = max(math.degrees(blur * up / (rad * DIAL_R)), 1e-3)
                t = 0.5 * (_erf((x[ok] + hw) / (math.sqrt(2.0) * sb)) -
                           _erf((x[ok] - hw) / (math.sqrt(2.0) * sb)))
                tmpl.append(t)
                obs.append(p[ok] - base)
            if not tmpl:
                return 1.0
            T = np.concatenate(tmpl)
            O = np.concatenate(obs)
            denom = float(T @ T)
            if denom <= 1e-12:
                continue
            a = float(T @ O) / denom
            if a <= 0:
                continue
            res = O - a * T
            best = min(best, float(res @ res) / (a * a * denom))
        return best

    def _visibility(self, rgrey, rsat, maskf, white, needle_ang, up):
        """Fraction of the dial face annulus that looks like unobstructed face.

        Only the *geometric footprint* of the detected needle (its predicted angular width at
        each radius + blur + 1 deg) is excluded - not a fixed wedge - so a dark object next
        to the needle still lowers visibility (Astra F1)."""
        vis_ok = (cv2.remap(rgrey, self._vis_x, self._vis_y, cv2.INTER_LINEAR) >= 0.70 * white) & \
                 (cv2.remap(rsat, self._vis_x, self._vis_y, cv2.INTER_LINEAR) < 70) & \
                 (cv2.remap(maskf, self._vis_x, self._vis_y, cv2.INTER_NEAREST) > 0.5)
        use = np.ones_like(vis_ok, bool)
        if needle_ang is not None:
            dang = np.abs((self._vis_ang - (needle_ang % 360.0) + 180.0) % 360.0 - 180.0)
            foot = (needle_halfwidth_deg(self._vis_r) +
                    np.degrees(1.5 * up / (self._vis_r * DIAL_R)) + 1.0)
            use &= dang > foot
        for lo, hi in VIS_EXCLUDE:
            use &= ~((self._vis_ang >= lo) & (self._vis_ang <= hi))
        return float(vis_ok[use].mean()) if use.any() else 0.0

    # ------------------------------------------------------------------ rendering helpers
    @staticmethod
    def rectify(bgr: np.ndarray, H: np.ndarray) -> np.ndarray:
        return cv2.warpPerspective(bgr, H, (TEX_W, TEX_H),
                                   flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)


def _robust_spread(vals: Sequence[float]) -> float:
    if len(vals) < 2:
        return 0.0
    v = np.asarray(vals, float)
    if len(v) == 2:
        return float(abs(v[0] - v[1]) / math.sqrt(2.0))
    return float(1.4826 * np.median(np.abs(v - np.median(v))))


def fuse(frames: List[FrameReading], frames_requested: int,
         params: FuseParams = FuseParams()) -> Observation:
    """Fuse a burst of per-frame readings into one observation with sigma and quality flags."""
    flags: List[str] = []
    received = len(frames)
    tagged = [f for f in frames if f.tag_found]
    valid = [f for f in tagged if f.valid]           # finite value + minimum confidence
    vals = [float(f.value) for f in valid]
    nan = float('nan')

    def med(xs, default=nan):
        xs = [x for x in xs if x is not None]
        return float(np.median(xs)) if xs else default

    vis = med([f.dial_visibility for f in tagged], 0.0)
    conf = med([f.needle_confidence for f in tagged], 0.0)
    support = med([f.needle_support for f in tagged], 0.0)
    ambiguity = med([f.needle_ambiguity for f in tagged], 1.0)
    residual = med([f.needle_residual for f in tagged], 1.0)
    out_of_range = any(f.out_of_range for f in tagged)
    view = med([f.view_angle_deg for f in tagged])
    dist = med([f.distance_m for f in tagged])
    radius = med([f.dial_radius_px for f in tagged], 0.0)
    radius_minor = med([f.dial_radius_minor_px for f in tagged], 0.0)
    sharp = med([f.sharpness for f in tagged], 0.0)
    spread = _robust_spread(vals)

    if received == 0:
        flags.append('NO_IMAGE')
    elif not tagged:
        flags.append('TAG_NOT_VISIBLE')
    else:
        if len(tagged) < received:
            flags.append('TAG_INTERMITTENT')
        if vis < params.min_visibility:
            flags.append('OCCLUDED')
        if not math.isnan(view) and view > params.max_view_angle_deg:
            flags.append('OBLIQUE_VIEW')
        if radius_minor < params.min_dial_radius_px:
            flags.append('TOO_FAR')
        if conf < params.min_needle_confidence:
            flags.append('LOW_NEEDLE_CONTRAST')
        if ambiguity >= AMBIGUITY_LIMIT or residual > MAX_MODEL_RESIDUAL:
            flags.append('AMBIGUOUS_NEEDLE')
        if support < MIN_NEEDLE_SUPPORT:
            flags.append('NEEDLE_PARTIAL')
        if out_of_range:
            flags.append('OUT_OF_SCALE')
        if len(vals) >= 2 and spread > params.max_frame_spread:
            flags.append('INCONSISTENT_FRAMES')
        if sharp < params.min_sharpness:
            flags.append('BLURRED')

    has_value = len(vals) > 0
    value = float(np.median(vals)) if has_value else nan
    sigma = nan
    if has_value:
        # physical validity: the printed scale is [VALUE_MIN, VALUE_MAX]; report the measured
        # value unchanged (never clamp) but flag it
        if ((value < VALUE_MIN - OUT_OF_SCALE_TOL or value > VALUE_MAX + OUT_OF_SCALE_TOL)
                and 'OUT_OF_SCALE' not in flags):
            flags.append('OUT_OF_SCALE')
        n = len(vals)
        rand = 1.25 * spread / math.sqrt(n) if n >= 2 else params.max_frame_spread
        # angular resolution at the needle sampling radius (0.35 R) in the foreshortened direction
        res_deg = math.degrees(NEEDLE_LOCALISATION_PX / max(0.35 * max(radius_minor, 1.0), 1e-6))
        sig_res = res_deg * VALUE_PER_DEG
        sigma = math.sqrt(rand ** 2 + sig_res ** 2 + params.sigma_floor ** 2)
        if not (math.isfinite(value) and math.isfinite(sigma) and sigma > 0):
            has_value, value, sigma = False, nan, nan    # defensive: never emit invalid numbers

    if not flags and has_value:
        detail = f'{value:.2f} +/- {sigma:.2f} {UNIT} from {len(vals)}/{received} frames'
    elif has_value:
        detail = f'{value:.2f} +/- {sigma:.2f} {UNIT} but flagged: {", ".join(flags)}'
    else:
        detail = 'no reading: ' + (', '.join(flags) if flags else 'needle not found')
    return Observation(has_value, value, sigma, flags, frames_requested, received, len(tagged),
                       len(vals), [round(v, 4) for v in vals], spread, conf, vis, view, dist,
                       radius, sharp, detail, radius_minor, support, ambiguity, residual)


def annotate(bgr: np.ndarray, fr: FrameReading, obs: Optional[Observation], title: str) -> np.ndarray:
    """Draw tag, projected dial, needle estimate and the fused result on a copy of the frame."""
    out = bgr.copy()
    if fr.tag_corners is not None:
        cv2.polylines(out, [fr.tag_corners.astype(np.int32)], True, (0, 255, 0), 2)
    if fr.homography is not None:
        H = fr.homography
        ring = np.array([_polar_xy(np.array(1.0), np.array(a)) for a in range(0, 361, 10)],
                        np.float32).reshape(-1, 1, 2)
        cv2.polylines(out, [cv2.perspectiveTransform(ring, H).astype(np.int32)], True,
                      (255, 200, 0), 1, cv2.LINE_AA)
        if fr.angle_deg is not None:
            seg = np.array([_polar_xy(np.array(0.0), np.array(fr.angle_deg)),
                            _polar_xy(np.array(0.85), np.array(fr.angle_deg))],
                           np.float32).reshape(-1, 1, 2)
            p = cv2.perspectiveTransform(seg, H).reshape(-1, 2).astype(int)
            cv2.line(out, tuple(p[0]), tuple(p[1]), (255, 0, 255), 2, cv2.LINE_AA)
    cv2.rectangle(out, (0, 0), (out.shape[1], 58), (30, 30, 30), -1)
    cv2.putText(out, title, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2,
                cv2.LINE_AA)
    line2 = obs.detail if obs is not None else fr.detail
    color = (90, 220, 90) if obs is not None and obs.has_value and not obs.flags else (0, 165, 255)
    cv2.putText(out, line2[:90], (8, 47), cv2.FONT_HERSHEY_SIMPLEX, 0.47, color, 1, cv2.LINE_AA)
    return out


def timed_read(reader: GaugeReader, bgr, tag_id, K):
    t0 = time.perf_counter()
    fr = reader.read_frame(bgr, tag_id, K)
    return fr, (time.perf_counter() - t0) * 1000.0
