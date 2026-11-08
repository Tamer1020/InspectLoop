"""Viewpoint generation and selection for gauge observation (ROS independent).

Candidates are robot poses on arcs in front of the gauge: standoff distances x view angles
measured from the dial normal, each facing the dial. A candidate is *feasible* when the
robot footprint (+ clearance) lies entirely in known free space of the occupancy map.
Visibility is NOT predicted from the 2D map (it has no heights, and overhanging occluders
are invisible to the 2D LiDAR); it is established by actually observing.

Selection
    initial(): best prior (frontal, nominal standoff).
    next(hint): untried, not-too-similar candidate maximising  prior + hint bonus:
        different_side  + angular separation from all tried views   (occlusion, hidden tag)
        frontal         + reduction of |view angle|                   (too oblique)
        closer          + shorter standoff on the same line of sight  (too few pixels)
    Directions whose line of sight was found blocked are excluded entirely
    (mark_blocked_direction), at every distance.
"""
from dataclasses import dataclass, field
import math
from typing import List, Optional, Sequence

import numpy as np


@dataclass(frozen=True)
class Viewpoint:
    index: int
    x: float
    y: float
    yaw: float
    distance: float
    angle_deg: float

    @property
    def label(self) -> str:
        return f'V{self.index} d={self.distance:.2f} a={self.angle_deg:+.0f}deg'

    def as_dict(self):
        return {'index': self.index, 'label': self.label, 'x': round(self.x, 3),
                'y': round(self.y, 3), 'yaw': round(self.yaw, 4), 'distance': self.distance,
                'angle_deg': self.angle_deg}


@dataclass
class OccupancyMap:
    """Row 0 = lowest y (ROS OccupancyGrid convention). Values: 0 free, 100 occupied, -1 unknown."""
    data: np.ndarray
    resolution: float
    origin_x: float
    origin_y: float

    def is_free_disc(self, x: float, y: float, radius: float, free_max: int = 25) -> bool:
        h, w = self.data.shape
        r = int(math.ceil(radius / self.resolution))
        c0 = int((x - self.origin_x) / self.resolution)
        r0 = int((y - self.origin_y) / self.resolution)
        if r0 - r < 0 or c0 - r < 0 or r0 + r >= h or c0 + r >= w:
            return False
        yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
        disc = yy * yy + xx * xx <= r * r
        patch = self.data[r0 - r:r0 + r + 1, c0 - r:c0 + r + 1]
        cells = patch[disc]
        return bool(np.all((cells >= 0) & (cells <= free_max)))

    @classmethod
    def from_pgm(cls, img: np.ndarray, resolution: float, origin: Sequence[float]):
        """Build from a map_server style image (row 0 = top, 0 = occupied, 254 = free)."""
        data = np.where(img < 50, 100, np.where(img > 250, 0, -1)).astype(np.int16)[::-1, :]
        return cls(data, resolution, float(origin[0]), float(origin[1]))


@dataclass
class ViewpointPlanner:
    gauge_x: float
    gauge_y: float
    normal_yaw: float
    occupancy: Optional[OccupancyMap]
    distances: Sequence[float] = (1.5, 1.2, 1.9)
    angles_deg: Sequence[float] = (0.0, -30.0, 30.0, -45.0, 45.0)
    clearance_m: float = 0.45
    nominal_distance: float = 1.5
    tried: List[Viewpoint] = field(default_factory=list)
    blocked_angles: List[float] = field(default_factory=list)
    blocked_sector_deg: float = 20.0

    def __post_init__(self):
        self.candidates: List[Viewpoint] = []
        self.rejected: List[Viewpoint] = []
        i = 0
        for d in self.distances:
            for a in self.angles_deg:
                th = self.normal_yaw + math.radians(a)
                x = self.gauge_x + d * math.cos(th)
                y = self.gauge_y + d * math.sin(th)
                yaw = math.atan2(math.sin(th + math.pi), math.cos(th + math.pi))
                vp = Viewpoint(i, x, y, yaw, d, a)
                i += 1
                if self.occupancy is None or self.occupancy.is_free_disc(x, y, self.clearance_m):
                    self.candidates.append(vp)
                else:
                    self.rejected.append(vp)

    def prior(self, vp: Viewpoint) -> float:
        return -abs(vp.angle_deg) / 45.0 - abs(vp.distance - self.nominal_distance) / 0.6

    def _similar(self, a: Viewpoint, b: Viewpoint) -> bool:
        return abs(a.angle_deg - b.angle_deg) < 10.0 and abs(a.distance - b.distance) < 0.15

    def available(self) -> List[Viewpoint]:
        return [c for c in self.candidates
                if not any(self._similar(c, t) for t in self.tried)
                and not any(abs(c.angle_deg - a) < self.blocked_sector_deg
                            for a in self.blocked_angles)]

    def mark_blocked_direction(self, vp: Viewpoint):
        """Line of sight from this direction failed (occlusion): exclude the whole angular
        sector, at every standoff distance - moving closer along a blocked ray does not help."""
        self.blocked_angles.append(vp.angle_deg)

    def initial(self) -> Optional[Viewpoint]:
        av = self.available()
        return max(av, key=lambda v: (self.prior(v), -v.index)) if av else None

    def mark_tried(self, vp: Viewpoint):
        self.tried.append(vp)

    def next(self, hint: str = 'any', reference: Optional[Viewpoint] = None) -> Optional[Viewpoint]:
        av = self.available()
        if not av:
            return None
        if not self.tried:
            return self.initial()

        def score(v: Viewpoint) -> float:
            s = self.prior(v)
            if hint == 'different_side':
                s += min(abs(v.angle_deg - t.angle_deg) for t in self.tried) / 30.0
            elif hint == 'frontal':
                s += (min(abs(t.angle_deg) for t in self.tried) - abs(v.angle_deg)) / 30.0
            elif hint == 'closer':
                ref = reference or self.tried[-1]
                # same line of sight (it worked), shorter standoff
                s += (ref.distance - v.distance) / 0.3 - abs(v.angle_deg - ref.angle_deg) / 15.0
            return s

        return max(av, key=lambda v: (score(v), -v.index))
