#!/usr/bin/env python3
"""Sample /odom for a few seconds and print the robot speed over time (fault-test evidence)."""
import math
import sys
import time

from nav_msgs.msg import Odometry
import rclpy
from rclpy.qos import qos_profile_sensor_data


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    rclpy.init()
    node = rclpy.create_node('odom_sampler')
    samples = []
    state = {'t0': None}

    def cb(m):
        now = time.monotonic()
        if state['t0'] is None:
            state['t0'] = now
            print('READY', flush=True)        # sampling window starts at the first message
        v = math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        samples.append((now - state['t0'], v, m.twist.twist.angular.z))

    node.create_subscription(Odometry, '/odom', cb, qos_profile_sensor_data)
    t_start = time.monotonic()
    while rclpy.ok() and time.monotonic() - t_start < duration + 20.0:
        rclpy.spin_once(node, timeout_sec=0.05)
        if state['t0'] is not None and time.monotonic() - state['t0'] >= duration:
            break
    for lo in [x * 0.5 for x in range(int(duration * 2))]:
        win = [s for s in samples if lo <= s[0] < lo + 0.5]
        if win:
            print(f't+{lo:.1f}-{lo + 0.5:.1f}s  max|v|={max(s[1] for s in win):.3f} m/s  '
                  f'max|w|={max(abs(s[2]) for s in win):.3f} rad/s  (n={len(win)})')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
