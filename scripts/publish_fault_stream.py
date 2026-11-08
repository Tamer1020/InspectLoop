#!/usr/bin/env python3
"""Publish a *present but invalid* sensor stream for fault-injection tests (Astra F3/F5).

  scan_nan      /scan at 10 Hz, every range NaN          (data arrives, rate looks healthy)
  scan_empty    /scan at 10 Hz with an empty ranges array
  image_frozen  /camera/image_raw at 10 Hz: one captured frame re-stamped with the current time
  image_future  /camera/image_raw at 10 Hz: live content but stamped 5 s in the future

The image modes copy one real frame from the live camera during the first `start_delay` seconds
(argv[2], default 2 s) and only then start publishing; scripts/fault_test.sh stops the real
camera bridge in between.
"""
import copy
import math
import sys
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan


def main():
    mode = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node('fault_stream_' + mode,
                             parameter_overrides=[rclpy.parameter.Parameter(
                                 'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    first = {'done': False}

    def announce():
        if not first['done']:
            first['done'] = True
            print('FIRST_PUBLISH', flush=True)

    if mode.startswith('scan'):
        pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)

        def tick():
            m = LaserScan()
            m.header.stamp = node.get_clock().now().to_msg()
            m.header.frame_id = 'lidar_link'
            m.angle_min, m.angle_max = -math.pi, math.pi
            m.angle_increment = 2 * math.pi / 719
            m.range_min, m.range_max = 0.12, 12.0
            m.ranges = [] if mode == 'scan_empty' else [float('nan')] * 720
            pub.publish(m)
            announce()
    else:
        pub = node.create_publisher(Image, '/camera/image_raw', 10)
        state = {'frame': None}
        start = time.monotonic() + (float(sys.argv[2]) if len(sys.argv) > 2 else 10.0)

        def on_img(msg):  # noqa: E306            # grab one real frame while the real camera still runs
            if state['frame'] is None and time.monotonic() < start:
                state['frame'] = msg
                print('CAPTURED', flush=True)

        node.create_subscription(Image, '/camera/image_raw', on_img, qos_profile_sensor_data)

        def tick():
            if state['frame'] is None or time.monotonic() < start:
                return              # the real camera bridge is stopped meanwhile
            m = copy.deepcopy(state['frame'])
            now = node.get_clock().now()
            if mode == 'image_future':
                now = now + rclpy.duration.Duration(seconds=5.0)
            m.header.stamp = now.to_msg()
            pub.publish(m)
            announce()
    node.create_timer(0.1, tick)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
