"""Start the inspection mission manager."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('inspectloop_mission')
    return LaunchDescription([
        DeclareLaunchArgument('mission_file',
                              default_value=os.path.join(pkg, 'config', 'gauges.yaml')),
        DeclareLaunchArgument('output_root',
                              default_value=os.path.join(os.path.expanduser('~'), 'inspectloop_runs')),
        Node(package='inspectloop_mission', executable='mission_manager', name='mission_manager',
             output='screen', emulate_tty=True,
             parameters=[os.path.join(pkg, 'config', 'mission_params.yaml'),
                         {'mission_file': LaunchConfiguration('mission_file'),
                          'output_root': LaunchConfiguration('output_root')}]),
    ])
