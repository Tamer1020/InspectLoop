#!/usr/bin/env python3
"""F3 live injector: /scan at 10 Hz (wall timer, sim-time stamps), 720 beams of 5 m,
angle_increment = 1e-9 and a matching 719e-9 rad span (the audit's malformed metadata)."""
import rclpy
import rclpy.clock
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

rclpy.init()
node = rclpy.create_node('fault_stream_tiny_inc', parameter_overrides=[
    rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
n = {'k': 0}


def tick():
    m = LaserScan()
    m.header.stamp = node.get_clock().now().to_msg()
    m.header.frame_id = 'lidar_link'
    m.angle_min, m.angle_increment = 0.0, 1e-9
    m.angle_max = 719 * 1e-9
    m.range_min, m.range_max = 0.12, 12.0
    m.ranges = [5.0] * 720
    pub.publish(m)
    n['k'] += 1
    if n['k'] == 1:
        print('FIRST_PUBLISH', flush=True)
    elif n['k'] % 50 == 0:
        print(f"PUBLISHED {n['k']}", flush=True)


node.create_timer(0.1, tick, clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
try:
    rclpy.spin(node)
except KeyboardInterrupt:
    pass
