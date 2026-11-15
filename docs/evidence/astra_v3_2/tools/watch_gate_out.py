#!/usr/bin/env python3
"""Record /cmd_vel_safe (the gate's 20 Hz timer output) for D wall seconds.
Prints READY at the first message, then per-0.5 s windows: count, max gap, max |v|, max |w|."""
import sys
import time

from geometry_msgs.msg import Twist
import rclpy

dur = float(sys.argv[1])
rclpy.init()
node = rclpy.create_node('gate_out_watch')
rec = []


def cb(m):
    if not rec:
        print('READY', flush=True)
    rec.append((time.monotonic(), abs(m.linear.x), abs(m.angular.z)))


node.create_subscription(Twist, '/cmd_vel_safe', cb, 50)
t_end = time.monotonic() + dur + 20.0
while rclpy.ok() and time.monotonic() < t_end:
    rclpy.spin_once(node, timeout_sec=0.01)
    if rec and rec[-1][0] - rec[0][0] >= dur:
        break
t0 = rec[0][0]
prev = None
for lo in [x * 0.5 for x in range(int(dur * 2))]:
    win = [r for r in rec if lo <= r[0] - t0 < lo + 0.5]
    if not win:
        print(f't+{lo:.1f}s  NO OUTPUT')
        continue
    gaps = [b[0] - a[0] for a, b in zip(rec, rec[1:]) if lo <= b[0] - t0 < lo + 0.5]
    print(f't+{lo:.1f}-{lo + 0.5:.1f}s  n={len(win)}  max_gap={1e3 * max(gaps or [0]):.0f}ms  '
          f'max|v|={max(r[1] for r in win):.3f}  max|w|={max(r[2] for r in win):.3f}')
allgaps = [b[0] - a[0] for a, b in zip(rec, rec[1:])]
print(f'TOTAL n={len(rec)} max_gap_ms={1e3 * max(allgaps):.0f}', flush=True)
