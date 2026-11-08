"""Start Gazebo Harmonic with the warehouse, spawn InspectLoop and bridge it to ROS 2."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (AppendEnvironmentVariable, DeclareLaunchArgument, EmitEvent,
                            ExecuteProcess, IncludeLaunchDescription, LogInfo,
                            RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gz_pkg = get_package_share_directory('inspectloop_gazebo')
    desc_pkg = get_package_share_directory('inspectloop_description')

    headless = LaunchConfiguration('headless')
    world = LaunchConfiguration('world')

    # Supervised wrapper: gz sim runs in its own process group which is torn down on shutdown
    # (the plain gz-tools wrapper can orphan the server process).
    gazebo = ExecuteProcess(
        cmd=[os.path.join(gz_pkg, 'scripts', 'gz_sim_supervised.sh'), headless, world],
        name='gazebo', output='screen', shell=False, sigterm_timeout='8')

    def _on_gazebo_exit(event, context):
        # Gazebo exiting on its own (crash / window closed) stops the whole system. During a
        # normal shutdown it exits too - emitting a second Shutdown then makes launch_ros fail
        # ("Cannot shutdown a ROS adapter that is not running"), so only react when running.
        if context.is_shutdown:
            return None
        return [LogInfo(msg='Gazebo exited - shutting down InspectLoop'),
                EmitEvent(event=Shutdown(reason='gazebo exited'))]

    gazebo_exit = RegisterEventHandler(OnProcessExit(target_action=gazebo,
                                                     on_exit=_on_gazebo_exit))

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(desc_pkg, 'launch', 'description.launch.py')),
        launch_arguments={'use_sim_time': 'true'}.items())

    spawn = Node(
        package='ros_gz_sim', executable='create', name='spawn_inspectloop', output='screen',
        arguments=['-name', 'inspectloop', '-topic', 'robot_description',
                   '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                   '-z', '0.02', '-Y', LaunchConfiguration('yaw')])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_bridge', output='screen',
        parameters=[{'config_file': os.path.join(gz_pkg, 'config', 'bridge.yaml'),
                     'use_sim_time': True}])

    lidar_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_lidar_bridge',
        output='screen',
        parameters=[{'config_file': os.path.join(gz_pkg, 'config', 'lidar_bridge.yaml'),
                     'use_sim_time': True}])

    cam_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='gz_camera_bridge',
        output='screen', condition=IfCondition(LaunchConfiguration('camera_bridge')),
        parameters=[{'config_file': os.path.join(gz_pkg, 'config', 'camera_bridge.yaml'),
                     'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false',
                              description='Run gz server only with offscreen rendering'),
        DeclareLaunchArgument('world', default_value=os.path.join(gz_pkg, 'worlds', 'warehouse.sdf')),
        DeclareLaunchArgument('camera_bridge', default_value='true'),
        DeclareLaunchArgument('x', default_value='-10.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(gz_pkg, 'models')),
        gazebo, gazebo_exit, rsp, spawn, bridge, lidar_bridge, cam_bridge,
    ])
