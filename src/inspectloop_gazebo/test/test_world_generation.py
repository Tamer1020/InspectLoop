"""The committed world, panel models and map must match the layout file."""
import filecmp
import os
import subprocess
import sys
import xml.etree.ElementTree as ET

import cv2
import numpy as np
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NAV_MAPS = os.path.join(os.path.dirname(PKG), 'inspectloop_navigation', 'maps')
LAYOUT = os.path.join(PKG, 'config', 'warehouse_layout.yaml')
GEN = os.path.join(PKG, 'scripts', 'generate_warehouse.py')


def _generate(tmp_path):
    subprocess.run([sys.executable, GEN, '--layout', LAYOUT,
                    '--world-out', str(tmp_path / 'warehouse.sdf'),
                    '--models-out', str(tmp_path / 'models'),
                    '--map-out', str(tmp_path / 'warehouse')], check=True)


def test_committed_outputs_are_up_to_date(tmp_path):
    _generate(tmp_path)
    assert filecmp.cmp(tmp_path / 'warehouse.sdf', os.path.join(PKG, 'worlds', 'warehouse.sdf'),
                       shallow=False), 'world out of date: re-run generate_warehouse.py'
    for ext in ('pgm', 'yaml'):
        assert filecmp.cmp(tmp_path / f'warehouse.{ext}', os.path.join(NAV_MAPS, f'warehouse.{ext}'),
                           shallow=False), f'map .{ext} out of date'


def test_world_is_wellformed_xml(tmp_path):
    _generate(tmp_path)
    root = ET.parse(tmp_path / 'warehouse.sdf').getroot()
    names = [m.get('name') for m in root.iter('model')]
    assert len(names) == len(set(names))
    assert 'floor' in names


def test_gauges_and_occluders_in_world(tmp_path):
    _generate(tmp_path)
    text = (tmp_path / 'warehouse.sdf').read_text()
    for tag in (11, 12, 13, 14):
        assert f'model://gauge_panel_{tag}' in text
    assert 'conduit_box_pg12' in text and 'crate_stack_pg13' in text


def _cell(img, layout, x, y):
    res = layout['map']['resolution']
    ox, oy = layout['map']['origin']
    col = int((x - ox) / res)
    row = img.shape[0] - 1 - int((y - oy) / res)
    return img[row, col]


def test_map_marks_obstacles_and_free_space(tmp_path):
    _generate(tmp_path)
    layout = yaml.safe_load(open(LAYOUT))
    img = cv2.imread(str(tmp_path / 'warehouse.pgm'), cv2.IMREAD_GRAYSCALE)
    sp = layout['robot_spawn']
    assert _cell(img, layout, sp['x'], sp['y']) == 254        # spawn is free
    for r in layout['racks']['items']:
        assert _cell(img, layout, *r['xy']) == 0               # rack centres occupied
    assert _cell(img, layout, -11.9, 0.0) == 0                 # west wall occupied
    assert set(np.unique(img)) <= {0, 254}


def test_overhanging_occluder_is_invisible_to_lidar_map(tmp_path):
    """The conduit box hangs above the LiDAR plane: in the world, NOT in the 2D map."""
    _generate(tmp_path)
    layout = yaml.safe_load(open(LAYOUT))
    img = cv2.imread(str(tmp_path / 'warehouse.pgm'), cv2.IMREAD_GRAYSCALE)
    occ = {o['name']: o for o in layout['occluders']}
    assert _cell(img, layout, *occ['conduit_box_pg12']['xy']) == 254
    assert _cell(img, layout, *occ['crate_stack_pg13']['xy']) == 0


def test_gauge_mounted_on_a_surface_not_inside_it(tmp_path):
    """Regression: a panel at x=11.89 was buried in the 0.2 m east wall (inner face 11.8)."""
    _generate(tmp_path)
    layout = yaml.safe_load(open(LAYOUT))
    img = cv2.imread(str(tmp_path / 'warehouse.pgm'), cv2.IMREAD_GRAYSCALE)
    for g in layout['gauges']['items']:
        x = g['xy'][0] + 0.10 * np.cos(g['normal_yaw'])
        y = g['xy'][1] + 0.10 * np.sin(g['normal_yaw'])
        assert _cell(img, layout, x, y) == 254, f"{g['id']}: space in front of the dial is not free"


def test_gauge_texture_contains_tag_and_dial(tmp_path):
    _generate(tmp_path)
    tex = cv2.imread(str(tmp_path / 'models' / 'gauge_panel_13' / 'materials' / 'textures' /
                         'gauge_13.png'))
    assert tex.shape[:2] == (680, 500)
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    _, ids, _ = cv2.aruco.detectMarkers(cv2.cvtColor(tex, cv2.COLOR_BGR2GRAY), d)
    assert ids is not None and ids.flatten().tolist() == [13]
