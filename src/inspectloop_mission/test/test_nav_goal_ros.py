"""Astra F4 against a real rclpy NavigateToPose action server (real goal/cancel protocol).

Behaviour is selected by goal.pose.pose.position.x:
  1 succeed after 0.3 s     2 reject            3 run until cancelled (cancel accepted)
  4 run, cancel REJECTED    5 accept only after 1.5 s (late acceptance), then run until cancelled
"""
import os
import threading
import time

from nav2_msgs.action import NavigateToPose
import pytest
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from inspectloop_mission.nav_goal import NavGoalRunner


# unique action name: never collide with a real Nav2 running on the same ROS graph
ACTION = f'test_navigate_to_pose_{os.getpid()}'


class FakeNav2:
    def __init__(self, node, name=ACTION):
        self.active = set()
        self.stop = threading.Event()
        self.server = ActionServer(node, NavigateToPose, name, self.execute,
                                   goal_callback=self.on_goal, cancel_callback=self.on_cancel,
                                   callback_group=ReentrantCallbackGroup())

    def on_goal(self, goal):
        mode = int(goal.pose.pose.position.x)
        if mode == 2:
            return GoalResponse.REJECT
        if mode == 5:
            time.sleep(1.5)
        return GoalResponse.ACCEPT

    def on_cancel(self, handle):
        mode = int(handle.request.pose.pose.position.x)
        return CancelResponse.REJECT if mode == 4 else CancelResponse.ACCEPT

    def execute(self, handle):
        mode = int(handle.request.pose.pose.position.x)
        key = id(handle)
        self.active.add(key)
        try:
            if mode == 1:
                time.sleep(0.3)
                handle.succeed()
                return NavigateToPose.Result()
            while not self.stop.is_set():
                if handle.is_cancel_requested:
                    handle.canceled()
                    return NavigateToPose.Result()
                time.sleep(0.05)
            handle.abort()
            return NavigateToPose.Result()
        finally:
            self.active.discard(key)


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    server_node = rclpy.create_node('fake_nav2')
    client_node = rclpy.create_node('nav_goal_test_client')
    fake = FakeNav2(server_node)
    client = ActionClient(client_node, NavigateToPose, ACTION,
                          callback_group=ReentrantCallbackGroup())
    ex = MultiThreadedExecutor(num_threads=6)
    ex.add_node(server_node)
    ex.add_node(client_node)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    assert client.wait_for_server(timeout_sec=5.0)
    yield fake, client
    fake.stop.set()
    time.sleep(0.3)
    ex.shutdown()
    rclpy.shutdown()


def wall_wait(fut, timeout, abort_check=None):
    end = time.monotonic() + timeout
    while not fut.done() and time.monotonic() < end:
        if abort_check is not None:
            abort_check()
        time.sleep(0.02)
    return fut.done()


def goal(mode):
    g = NavigateToPose.Goal()
    g.pose.header.frame_id = 'map'
    g.pose.pose.position.x = float(mode)
    return g


def make_runner(client):
    return NavGoalRunner(client, wall_wait, time.monotonic, None, accept_timeout_s=0.5,
                         cancel_timeout_s=2.0, terminal_timeout_s=3.0)


def test_real_success_and_rejection(ros):
    _, client = ros
    r = make_runner(client)
    assert r.run(goal(1), 'ok', 5.0)['result'] == 'SUCCEEDED'
    assert r.run(goal(2), 'rej', 5.0)['result'] == 'REJECTED'
    assert not r.motion_inhibited()


def test_real_timeout_cancel_confirmed(ros):
    fake, client = ros
    r = make_runner(client)
    out = r.run(goal(3), 'cancellable', 0.6)
    assert out['result'] == 'TIMEOUT' and out['cancel_confirmed'] is True
    assert not r.motion_inhibited()
    time.sleep(0.2)
    assert not fake.active


def test_real_cancel_rejected_inhibits_motion(ros):
    fake, client = ros
    r = make_runner(client)
    out = r.run(goal(4), 'stubborn', 0.6)
    assert out['result'] == 'TIMEOUT' and out['cancel_confirmed'] is False
    assert r.motion_inhibited()
    assert r.run(goal(1), 'next', 5.0)['result'] == 'INHIBITED'


def test_real_late_acceptance_is_cancelled(ros):
    fake, client = ros
    r = make_runner(client)
    assert r.run(goal(5), 'late', 5.0)['result'] == 'NO_RESPONSE'
    assert r.motion_inhibited()
    # while the request is still unanswered no new goal may be sent
    assert r.run(goal(1), 'too_early', 5.0)['result'] == 'INHIBITED'
    time.sleep(1.5)                                  # late acceptance arrives -> cancelled
    assert r.cancel_all('test') is True
    assert not r.motion_inhibited()
    assert r.run(goal(1), 'after_late', 5.0)['result'] == 'SUCCEEDED'
    assert any(e['event'] == 'LATE_ACCEPTANCE' for e in r.events)


def test_v1_client_logic_orphans_a_late_goal(ros):
    """Reproduction of the v1 behaviour (Astra F4): give up after the accept timeout and
    forget the request -> the late-accepted goal keeps executing on the server."""
    _, client = ros
    send = client.send_goal_async(goal(5))
    assert not wall_wait(send, 0.5)               # v1: 'NO_RESPONSE' and moves on
    time.sleep(2.0)
    assert send.done() and send.result().accepted
    result = send.result().get_result_async()
    time.sleep(0.5)
    assert not result.done()                       # orphan goal still executing on the server
    send.result().cancel_goal_async()             # clean up for the other tests
    assert wall_wait(result, 3.0)


def test_v3_interrupt_during_acceptance_real_server_goal_is_cancelled(ros):
    """V3 (Astra V2 review F4): KeyboardInterrupt inside the acceptance wait; the real server
    accepts 1.5 s later (after the bounded cleanup gave up). The goal must stay owned, block
    motion, and be cancelled + confirmed terminal on the server."""
    fake, client = ros
    before = set(fake.active)       # e.g. the deliberately un-cancellable goal of another test
    state = {'fired': False}

    def interrupting_wait(fut, timeout, abort_check=None):
        if not state['fired']:
            state['fired'] = True
            time.sleep(0.1)
            raise KeyboardInterrupt('ctrl-c during acceptance')
        return wall_wait(fut, timeout, abort_check)

    r = NavGoalRunner(client, interrupting_wait, time.monotonic, None, accept_timeout_s=0.5,
                      cancel_timeout_s=2.0, terminal_timeout_s=3.0)
    with pytest.raises(KeyboardInterrupt):
        r.run(goal(5), 'interrupted', 30.0)
    assert r.motion_inhibited()                    # request in flight: owned and inhibiting
    assert r.run(goal(1), 'next', 5.0)['result'] == 'INHIBITED'
    end = time.monotonic() + 5.0                   # late acceptance arrives on the server
    while (not any(e['event'] == 'LATE_ACCEPTANCE' for e in r.events) and
           time.monotonic() < end):
        r.unresolved()                             # polls the pending send
        time.sleep(0.05)
    ok = r.cancel_all('shutdown')
    assert ok is True
    time.sleep(0.3)
    assert fake.active <= before                   # our goal no longer executes on the server
    assert not r.motion_inhibited()
    kinds = [e['event'] for e in r.events]
    assert 'LATE_ACCEPTANCE' in kinds
    assert any(e['event'] == 'TERMINAL' and 'CANCELED' in e['detail'] for e in r.events), r.events
