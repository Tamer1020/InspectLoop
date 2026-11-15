#!/usr/bin/env python3
"""V3.2 F4 live harness: an UNKNOWN-ownership NavigateToPose goal against the real Nav2 stack.

Run with the simulation up and NO mission manager (inspectloop.launch.py mission:=false).
The NavGoalRunner sends a real goal to /navigate_to_pose; a KeyboardInterrupt is injected
(sys.settrace, same injector as the unit tests) right after send_goal_async() returned and
before its future was stored, so the runner holds no handle: ownership UNKNOWN.

  fallback    runner with GoalIdCanceller (V3.2): expect our goal id CANCELED on the server,
              robot stationary, cancel_all() True only after both are observed.
  nofallback  V3.1 behaviour (control): expect cancel_all() False, and the goal KEEPS
              EXECUTING on Nav2 (robot drives). The harness then cancels it by id to clean up.

Prints a timeline: Nav2 status of our goal id, robot speed, runner verdicts and events.
"""
import math
import os
import sys
import threading
import time

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import qos_profile_sensor_data

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', '..', '..', '..', 'src', 'inspectloop_mission', 'test'))
from inspectloop_mission.nav_goal import GoalIdCanceller, NavGoalRunner, random_goal_id  # noqa
from test_v3_1_nav_send_interrupt import InterruptAfterSend  # noqa

STATUS = {None: 'not listed', 1: 'ACCEPTED', 2: 'EXECUTING', 3: 'CANCELING', 4: 'SUCCEEDED',
          5: 'CANCELED', 6: 'ABORTED'}


class Spy:
    def __init__(self, c):
        self._c, self.returned = c, False

    def send_goal_async(self, g, feedback_callback=None, goal_uuid=None):
        f = self._c.send_goal_async(g, feedback_callback=feedback_callback, goal_uuid=goal_uuid)
        self.returned = True
        return f


def wall_wait(fut, timeout, abort_check=None):
    end = time.monotonic() + timeout
    while not fut.done() and time.monotonic() < end:
        time.sleep(0.02)
    return fut.done()


def main():
    mode = sys.argv[1]
    rclpy.init()
    node = rclpy.create_node('v32_unknown_goal_harness', parameter_overrides=[
        rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
    cg = ReentrantCallbackGroup()
    client = ActionClient(node, NavigateToPose, 'navigate_to_pose', callback_group=cg)
    canceller = GoalIdCanceller(node, 'navigate_to_pose', cg)
    odom = {'v': None, 'w': None, 't': 0.0}

    def on_odom(m):
        odom.update(v=math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y),
                    w=abs(m.twist.twist.angular.z), t=time.monotonic())
    node.create_subscription(Odometry, '/odom', on_odom, qos_profile_sensor_data,
                             callback_group=cg)
    ex = MultiThreadedExecutor(num_threads=4)
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    assert client.wait_for_server(timeout_sec=60.0), 'navigate_to_pose not available'
    t_end = time.monotonic() + 30
    while odom['v'] is None and time.monotonic() < t_end:
        time.sleep(0.1)

    def stopped():
        return (odom['v'] is not None and time.monotonic() - odom['t'] < 0.5 and
                odom['v'] < 0.02 and odom['w'] < 0.05)

    ids = []

    def new_id():
        ids.append(random_goal_id())
        return ids[-1]
    spy = Spy(client)
    r = NavGoalRunner(spy, wall_wait, time.monotonic, node.get_logger(), accept_timeout_s=5.0,
                      cancel_timeout_s=5.0, terminal_timeout_s=10.0,
                      goal_id_fallback=canceller if mode == 'fallback' else None,
                      motion_stopped=stopped if mode == 'fallback' else None, new_goal_id=new_id)
    g = NavigateToPose.Goal()
    g.pose = PoseStamped()
    g.pose.header.frame_id = 'map'
    g.pose.pose.position.x, g.pose.pose.position.y = -7.0, 2.49
    g.pose.pose.orientation.z, g.pose.pose.orientation.w = math.sin(math.pi / 4), math.cos(math.pi / 4)
    t0 = time.monotonic()
    log = lambda msg: print(f't+{time.monotonic() - t0:5.2f}s  {msg}', flush=True)  # noqa: E731
    log(f'mode={mode}: sending goal to (-7.00, 2.49) with SIGINT injected after send')
    with InterruptAfterSend(spy, nth=1, mode='opcode') as inj:
        try:
            r.run(g, 'LIVE', 60.0)
            log('run() returned without interruption (unexpected)')
        except KeyboardInterrupt as e:
            log(f'KeyboardInterrupt in run(): {e}')
    gid = ids[0]
    log(f'injection point {inj.where}; unresolved={r.unresolved()}')
    verdict = False
    for k in range(40):
        time.sleep(0.5)
        st = canceller.status(gid)
        v = odom['v']
        if k % 2 == 0 or st in (3, 5, 6, 4):
            log(f'nav2 status of our goal id: {STATUS.get(st, st)}  |v|={v:.3f} m/s')
        if k in (4, 12, 20, 28, 36):          # cleanup attempts at t+~2.5 s, 6.5 s, ...
            verdict = r.cancel_all('harness cleanup')
            log(f'cancel_all() -> {verdict}; unresolved={r.unresolved()}')
            if verdict and mode == 'fallback':
                break
    for _ in range(10):                        # 5 s of robot speed after the verdict
        time.sleep(0.5)
        log(f'after verdict: nav2 status {STATUS.get(canceller.status(gid))}  |v|={odom["v"]:.3f} m/s')
    for e in r.events:
        print('EVENT', e, flush=True)
    final = canceller.status(gid)
    print(f'RESULT mode={mode} cancel_all={verdict} nav2_status={STATUS.get(final, final)} '
          f'robot_stopped={stopped()}', flush=True)
    if mode == 'nofallback' and final not in (4, 5, 6):
        f = canceller.cancel(gid)              # harness cleanup of the orphan (not the runner)
        wall_wait(f, 5.0)
        time.sleep(2.0)
        print(f'HARNESS_CLEANUP cancel-by-id code={getattr(f.result(), "return_code", None)} '
              f'nav2_status={STATUS.get(canceller.status(gid))}', flush=True)
    ex.shutdown()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
