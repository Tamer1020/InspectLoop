"""Full InspectLoop system: Gazebo + robot + bridge, safety layer, Nav2, gauge reader,
uncertainty-aware mission manager and RViz.

    ros2 launch inspectloop_bringup inspectloop.launch.py               # GUI + RViz + mission
    ros2 launch inspectloop_bringup inspectloop.launch.py headless:=true rviz:=false
    ros2 launch inspectloop_bringup inspectloop.launch.py mission:=false   # manual goals in RViz
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gz = get_package_share_directory('inspectloop_gazebo')
    nav = get_package_share_directory('inspectloop_navigation')
    safety = get_package_share_directory('inspectloop_safety')
    vision = get_package_share_directory('inspectloop_vision')
    mission = get_package_share_directory('inspectloop_mission')

    lc = LaunchConfiguration

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz, 'launch', 'sim.launch.py')),
        launch_arguments={'headless': lc('headless'), 'camera_bridge': lc('camera_bridge')}.items())

    safety_cfg = os.path.join(safety, 'config', 'safety.yaml')
    gate = Node(package='inspectloop_safety', executable='cmd_vel_gate', name='cmd_vel_gate',
                output='screen', parameters=[safety_cfg])
    health = Node(package='inspectloop_safety', executable='health_monitor', name='health_monitor',
                  output='screen', parameters=[safety_cfg])

    navigation = TimerAction(period=5.0, actions=[IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav, 'launch', 'navigation.launch.py')))])

    gauge_reader = Node(package='inspectloop_vision', executable='gauge_reader_node',
                        name='gauge_reader', output='screen',
                        parameters=[os.path.join(vision, 'config', 'gauge_reader.yaml')])

    mission_node = Node(
        package='inspectloop_mission', executable='mission_manager', name='mission_manager',
        output='screen', emulate_tty=True, condition=IfCondition(lc('mission')),
        sigterm_timeout='40',      # allow goal cancellation to be confirmed on Ctrl-C (F4)
        parameters=[os.path.join(mission, 'config', 'mission_params.yaml'),
                    {'mission_file': lc('mission_file'), 'output_root': lc('output_root')}])
    mission_delayed = TimerAction(period=10.0, actions=[mission_node])

    shutdown_after_mission = RegisterEventHandler(
        OnProcessExit(target_action=mission_node, on_exit=[
            LogInfo(msg='Mission finished - report written. Shutting down (exit_on_mission_end).'),
            EmitEvent(event=Shutdown(reason='mission finished'))]),
        condition=IfCondition(lc('exit_on_mission_end')))

    rviz = Node(package='rviz2', executable='rviz2', name='rviz2', output='log',
                condition=IfCondition(lc('rviz')),
                arguments=['-d', os.path.join(nav, 'rviz', 'inspectloop.rviz')],
                parameters=[{'use_sim_time': True}])

    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('mission', default_value='true'),
        DeclareLaunchArgument('camera_bridge', default_value='true'),
        DeclareLaunchArgument('exit_on_mission_end', default_value='false'),
        DeclareLaunchArgument('mission_file',
                              default_value=os.path.join(mission, 'config', 'gauges.yaml')),
        DeclareLaunchArgument('output_root',
                              default_value=os.path.join(os.path.expanduser('~'), 'inspectloop_runs')),
        sim, gate, health, navigation, gauge_reader, mission_delayed, shutdown_after_mission, rviz,
    ])
