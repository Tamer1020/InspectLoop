"""Consistency checks between Nav2 parameters, the map and the robot."""
import os

import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
PARAMS = yaml.safe_load(open(os.path.join(PKG, 'config', 'nav2_params.yaml')))
LAYOUT = yaml.safe_load(open(os.path.join(os.path.dirname(PKG), 'inspectloop_gazebo', 'config',
                                          'warehouse_layout.yaml')))


def p(node):
    return PARAMS[node]['ros__parameters']


def test_amcl_initial_pose_matches_spawn():
    ip = p('amcl')['initial_pose']
    sp = LAYOUT['robot_spawn']
    assert p('amcl')['set_initial_pose'] is True
    assert (ip['x'], ip['y'], ip['yaw']) == (sp['x'], sp['y'], sp['yaw'])


def test_frames_are_consistent():
    assert p('amcl')['base_frame_id'] == 'base_footprint'
    assert p('amcl')['global_frame_id'] == 'map'
    assert p('amcl')['odom_frame_id'] == 'odom'
    for cm in ('local_costmap', 'global_costmap'):
        q = PARAMS[cm][cm]['ros__parameters']
        assert q['robot_base_frame'] in ('base_link', 'base_footprint')
    assert PARAMS['global_costmap']['global_costmap']['ros__parameters']['global_frame'] == 'map'
    assert PARAMS['local_costmap']['local_costmap']['ros__parameters']['global_frame'] == 'odom'


def test_robot_radius_covers_chassis():
    for cm in ('local_costmap', 'global_costmap'):
        r = PARAMS[cm][cm]['ros__parameters']['robot_radius']
        assert r >= 0.31  # chassis 0.50 x 0.38 m, wheels at y=+-0.25 m


def test_velocity_limits_consistent():
    fp = p('controller_server')['FollowPath']
    vs = p('velocity_smoother')
    assert vs['max_velocity'][0] >= fp['vx_max']
    assert fp['vx_max'] <= 0.6  # Gazebo DiffDrive limit


def test_amcl_range_within_lidar_spec():
    assert p('amcl')['laser_max_range'] <= 12.0
    assert p('amcl')['laser_min_range'] >= 0.12


def test_map_yaml_points_to_existing_image():
    m = yaml.safe_load(open(os.path.join(PKG, 'maps', 'warehouse.yaml')))
    assert os.path.exists(os.path.join(PKG, 'maps', m['image']))
    assert m['resolution'] == LAYOUT['map']['resolution']
