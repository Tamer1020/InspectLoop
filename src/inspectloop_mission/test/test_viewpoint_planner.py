"""Viewpoint planner on synthetic grids and on the real warehouse map."""
import math
import os

import cv2
import numpy as np
import yaml

from inspectloop_mission.mission_config import load_mission
from inspectloop_mission.viewpoint_planner import OccupancyMap, ViewpointPlanner

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', '..')
MAP_YAML = os.path.join(SRC, 'inspectloop_navigation', 'maps', 'warehouse.yaml')
MISSION = load_mission(os.path.join(HERE, '..', 'config', 'gauges.yaml'))

CAM_X, CAM_Z = 0.255, 0.18          # camera in base_footprint (URDF)
CAM_PITCH_UP, HFOV = 0.25, 1.047
VFOV = 2 * math.atan(math.tan(HFOV / 2) * 480 / 640)


def real_map():
    m = yaml.safe_load(open(MAP_YAML))
    img = cv2.imread(os.path.join(os.path.dirname(MAP_YAML), m['image']), cv2.IMREAD_GRAYSCALE)
    return OccupancyMap.from_pgm(img, m['resolution'], m['origin'])


def empty_map():
    return OccupancyMap(np.zeros((200, 200), np.int16), 0.05, -5.0, -5.0)


def test_candidates_face_the_gauge():
    p = ViewpointPlanner(0.0, 0.0, 0.0, empty_map())
    assert len(p.candidates) == 15
    for v in p.candidates:
        bearing = math.atan2(-v.y, -v.x)
        assert abs(math.atan2(math.sin(bearing - v.yaw), math.cos(bearing - v.yaw))) < 1e-6
        assert abs(math.hypot(v.x, v.y) - v.distance) < 1e-9


def test_initial_is_frontal_nominal():
    v = ViewpointPlanner(0.0, 0.0, 0.0, empty_map()).initial()
    assert v.angle_deg == 0.0 and v.distance == 1.5


def test_occupied_cells_reject_candidates():
    grid = empty_map()
    grid.data[:, :] = 0
    # wall of obstacles at x in [1.3, 1.7] -> frontal d=1.5 candidates blocked
    c0, c1 = int((1.3 + 5) / 0.05), int((1.7 + 5) / 0.05)
    grid.data[:, c0:c1] = 100
    p = ViewpointPlanner(0.0, 0.0, 0.0, grid)
    assert all(not (abs(v.x - 1.5) < 0.5) for v in p.candidates)
    assert p.rejected


def test_unknown_space_is_not_free():
    grid = empty_map()
    grid.data[:, :] = -1
    assert ViewpointPlanner(0.0, 0.0, 0.0, grid).candidates == []


def test_different_side_after_occlusion():
    p = ViewpointPlanner(0.0, 0.0, 0.0, empty_map())
    v0 = p.initial()
    p.mark_tried(v0)
    v1 = p.next('different_side')
    assert abs(v1.angle_deg) == 45.0
    p.mark_tried(v1)
    v2 = p.next('different_side')
    assert v2.angle_deg == -v1.angle_deg      # then the other side


def test_frontal_and_closer_hints():
    p = ViewpointPlanner(0.0, 0.0, 0.0, empty_map())
    first = [v for v in p.candidates if v.angle_deg == 45.0 and v.distance == 1.9][0]
    p.mark_tried(first)
    assert abs(p.next('frontal').angle_deg) < 45.0
    p2 = ViewpointPlanner(0.0, 0.0, 0.0, empty_map())
    p2.mark_tried(p2.initial())
    assert p2.next('closer').distance == 1.2


def test_exhaustion_returns_none():
    p = ViewpointPlanner(0.0, 0.0, 0.0, empty_map(), distances=[1.5], angles_deg=[0.0])
    p.mark_tried(p.initial())
    assert p.next('any') is None


def test_every_gauge_has_a_feasible_frontal_view_framing_the_dial():
    grid = real_map()
    for g in MISSION.gauges:
        p = ViewpointPlanner(g.x, g.y, g.normal_yaw, grid)
        v = p.initial()
        assert v is not None, g.id
        cx = v.x + CAM_X * math.cos(v.yaw)
        cy = v.y + CAM_X * math.sin(v.yaw)
        dist = math.hypot(g.x - cx, g.y - cy)
        # dial centre z = panel centre 0.46 - (465-340) px * 0.0008 m/px
        dial_z, dial_r = 0.46 - 125 * 0.0008, 0.152
        for z in (dial_z - dial_r, dial_z + dial_r):
            elev = math.atan2(z - CAM_Z, dist) - CAM_PITCH_UP
            assert abs(elev) < VFOV / 2, g.id
        tag_top = 0.46 + 0.272 - 40 * 0.0008
        assert math.atan2(tag_top - CAM_Z, dist) - CAM_PITCH_UP < VFOV / 2, g.id


def test_crate_blocks_close_views_of_pg13():
    grid = real_map()
    g = [x for x in MISSION.gauges if x.id == 'PG-13'][0]
    p = ViewpointPlanner(g.x, g.y, g.normal_yaw, grid)
    assert any(v.distance == 1.2 for v in p.rejected)
