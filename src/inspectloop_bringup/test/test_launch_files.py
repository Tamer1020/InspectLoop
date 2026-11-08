"""All InspectLoop launch files must import and build a LaunchDescription."""
import importlib.util
import os

import pytest
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription

LAUNCH_FILES = [
    ('inspectloop_bringup', 'inspectloop.launch.py'),
    ('inspectloop_gazebo', 'sim.launch.py'),
    ('inspectloop_navigation', 'navigation.launch.py'),
    ('inspectloop_navigation', 'rviz.launch.py'),
    ('inspectloop_description', 'description.launch.py'),
    ('inspectloop_mission', 'mission.launch.py'),
]


@pytest.mark.parametrize('pkg,name', LAUNCH_FILES)
def test_launch_description_builds(pkg, name):
    path = os.path.join(get_package_share_directory(pkg), 'launch', name)
    spec = importlib.util.spec_from_file_location(name.replace('.', '_'), path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    ld = mod.generate_launch_description()
    assert isinstance(ld, LaunchDescription)
    assert len(ld.entities) > 0
