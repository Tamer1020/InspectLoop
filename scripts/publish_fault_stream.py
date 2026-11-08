#!/usr/bin/env python3
"""Publish a *present but invalid* sensor stream for fault-injection tests (Astra F3/F5).

  scan_nan      /scan at 10 Hz, every range NaN          (data arrives, rate looks healthy)
  scan_empty    /scan at 10 Hz with an empty ranges array
  scan_front_nan  relays the LIVE scan from /scan_live to /scan with every beam inside +-35 deg
                set to NaN (V3, Astra V2 review F3: the rest of the scan stays valid, the rate
                stays healthy - only the forward stopping corridor is blind)
  image_frozen  /camera/image_raw at 10 Hz: one captured frame re-stamped with the current time
  image_future  /camera/image_raw at 10 Hz: live content but stamped 5 s in the future

The image modes copy the first real frame they receive from the live camera and start publishing
`delay` seconds later (argv[2], default 1 s); scripts/fault_test.sh stops the real camera bridge
as soon as the frame has been captured.
"""
import copy
import math
import sys
import time

import rclpy
import rclpy.clock
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, LaserScan


def main():
    mode = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node('fault_stream_' + mode,
                             parameter_overrides=[rclpy.parameter.Parameter(
                                 'use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    first = {'done': False, 'n': 0}

    def announce():
        first['n'] += 1
        if not first['done']:
            first['done'] = True
            print('FIRST_PUBLISH', flush=True)
        elif first['n'] % 50 == 0:
            print(f"PUBLISHED {first['n']}", flush=True)

    if mode == 'scan_front_nan':
        pub = node.create_publisher(LaserScan, '/scan', qos_profile_sensor_data)
        half = math.radians(35.0)

        def on_live(msg):
            m = copy.deepcopy(msg)
            r = list(m.ranges)
            for i in range(len(r)):
                if abs(m.angle_min + i * m.angle_increment) <= half:
                    r[i] = float('nan')
            m.ranges = r
            pub.publish(m)
            announce()
        node.create_subscription(LaserScan, '/scan_live', on_live, qos_profile_sensor_data)
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        return
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
        state = {'frame': None, 'start': None}
        delay = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0

        def on_img(msg):  # noqa: E306 - grab the first real frame, start publishing `delay` s later
            if state['frame'] is None:
                state['frame'] = msg
                state['start'] = time.monotonic() + delay
                print('CAPTURED', flush=True)

        node.create_subscription(Image, '/camera/image_raw', on_img, qos_profile_sensor_data)

        def tick():
            if state['frame'] is None or time.monotonic() < state['start']:
                return              # the real camera bridge is stopped meanwhile
            m = copy.deepcopy(state['frame'])
            now = node.get_clock().now()
            if mode == 'image_future':
                now = now + rclpy.duration.Duration(seconds=5.0)
            m.header.stamp = now.to_msg()
            pub.publish(m)
            announce()
    # V3: publish on a WALL-clock timer (stamps still come from the sim clock). A sim-time timer
    # in this Python process stalled once in the V3 batch (scan_nan run 1: the gate saw a scan
    # timeout instead of invalid scans), so the fault would not have been exercised.
    node.create_timer(0.1, tick, clock=rclpy.clock.Clock(clock_type=rclpy.clock.ClockType.STEADY_TIME))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
