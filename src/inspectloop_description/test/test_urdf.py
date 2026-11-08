"""Static validation of the robot description (runs without Gazebo)."""
import os
import subprocess
import xml.etree.ElementTree as ET

import pytest

PKG_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
XACRO = os.path.join(PKG_DIR, 'urdf', 'inspectloop_robot.urdf.xacro')


def _expand(sim_mode='true'):
    env = dict(os.environ)
    # Allow running from the source tree before installation.
    return subprocess.run(['xacro', XACRO, f'sim_mode:={sim_mode}'],
                          check=True, capture_output=True, text=True, env=env).stdout


@pytest.fixture(scope='module')
def urdf_root():
    return ET.fromstring(_expand())


def test_required_frames(urdf_root):
    links = {l.get('name') for l in urdf_root.findall('link')}
    for frame in ['base_footprint', 'base_link', 'left_wheel_link', 'right_wheel_link',
                  'lidar_link', 'camera_link', 'camera_optical_link']:
        assert frame in links, frame


def test_single_tree_rooted_at_base_footprint(urdf_root):
    parents = {}
    for j in urdf_root.findall('joint'):
        child = j.find('child').get('link')
        assert child not in parents, f'{child} has two parents'
        parents[child] = j.find('parent').get('link')
    links = {l.get('name') for l in urdf_root.findall('link')}
    roots = links - set(parents)
    assert roots == {'base_footprint'}


def test_drive_wheels_are_continuous_about_y(urdf_root):
    for side in ('left', 'right'):
        j = urdf_root.find(f"joint[@name='{side}_wheel_joint']")
        assert j.get('type') == 'continuous'
        assert j.find('axis').get('xyz').split() == ['0', '1', '0']


def test_wheel_separation_matches_plugin(urdf_root):
    ys = [float(urdf_root.find(f"joint[@name='{s}_wheel_joint']/origin").get('xyz').split()[1])
          for s in ('left', 'right')]
    sep = ys[0] - ys[1]
    plugin = urdf_root.find(".//plugin[@name='gz::sim::systems::DiffDrive']")
    assert plugin is not None
    assert abs(float(plugin.find('wheel_separation').text) - sep) < 1e-9
    assert abs(float(plugin.find('wheel_radius').text) - 0.08) < 1e-9


def test_all_non_virtual_links_have_positive_inertia(urdf_root):
    for link in urdf_root.findall('link'):
        inertial = link.find('inertial')
        if link.get('name') in ('base_footprint', 'camera_optical_link'):
            assert inertial is None
            continue
        assert inertial is not None, link.get('name')
        assert float(inertial.find('mass').get('value')) > 0
        i = inertial.find('inertia')
        ixx, iyy, izz = (float(i.get(k)) for k in ('ixx', 'iyy', 'izz'))
        assert min(ixx, iyy, izz) > 0
        # triangle inequality for physical inertia tensors
        assert ixx + iyy >= izz and ixx + izz >= iyy and iyy + izz >= ixx


def test_sensor_frames_match_tf(urdf_root):
    sensors = {s.get('name'): s for s in urdf_root.iter('sensor')}
    assert sensors['lidar'].find('gz_frame_id').text == 'lidar_link'
    assert sensors['inspection_camera'].find('gz_frame_id').text == 'camera_optical_link'


def test_non_sim_mode_has_no_gazebo_plugins():
    root = ET.fromstring(_expand('false'))
    assert root.find('.//plugin') is None
