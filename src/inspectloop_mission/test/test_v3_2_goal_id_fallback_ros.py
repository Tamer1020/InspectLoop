"""V3.2 (F4 hardening) against a real rclpy action server: GoalIdCanceller over the real
CancelGoal service and status topic. Verifies the protocol assumptions the runner relies on:
targeted cancellation by goal id, ERROR_UNKNOWN_GOAL_ID for an id the server never received,
terminal status published per goal id - and that another client's goal is never cancelled.
"""
import threading
import time

from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from nav2_msgs.action import NavigateToPose
import pytest
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from unique_identifier_msgs.msg import UUID

from inspectloop_mission.nav_goal import GoalIdCanceller, NavGoalRunner, random_goal_id
from test_nav_goal_ros import ACTION as BASE_ACTION, FakeNav2, goal, wall_wait  # noqa: I100
from test_v3_1_nav_send_interrupt import InterruptAfterSend  # noqa: I100

ACTION = BASE_ACTION + '_v32'    # own server name: no status cross-talk with other modules


class ReturnFlagClient:
    """Real ActionClient; sets `returned` when send_goal_async has returned (for injection)."""

    def __init__(self, client):
        self._c, self.returned = client, False

    def send_goal_async(self, g, feedback_callback=None, goal_uuid=None):
        f = self._c.send_goal_async(g, feedback_callback=feedback_callback, goal_uuid=goal_uuid)
        self.returned = True
        return f


@pytest.fixture(scope='module')
def ros():
    rclpy.init()
    server_node = rclpy.create_node('fake_nav2_v32')
    client_node = rclpy.create_node('nav_goal_v32_client')
    other_node = rclpy.create_node('foreign_client')       # e.g. RViz Nav2 panel
    fake = FakeNav2(server_node, ACTION)
    cg = ReentrantCallbackGroup()
    client = ActionClient(client_node, NavigateToPose, ACTION, callback_group=cg)
    other = ActionClient(other_node, NavigateToPose, ACTION, callback_group=cg)
    canceller = GoalIdCanceller(client_node, ACTION, cg)
    ex = MultiThreadedExecutor(num_threads=6)
    for n in (server_node, client_node, other_node):
        ex.add_node(n)
    th = threading.Thread(target=ex.spin, daemon=True)
    th.start()
    assert client.wait_for_server(timeout_sec=5.0) and other.wait_for_server(timeout_sec=5.0)
    yield fake, client, other, canceller
    fake.stop.set()
    time.sleep(0.3)
    ex.shutdown()
    rclpy.shutdown()


def wait_until(pred, timeout):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.02)
    return pred()


def test_lost_future_goal_cancelled_by_id_foreign_goal_untouched(ros):
    fake, client, other, canceller = ros
    fsend = other.send_goal_async(goal(3))                  # foreign goal: runs until cancelled
    assert wall_wait(fsend, 3.0) and fsend.result().accepted
    foreign_id = fsend.result().goal_id
    spy = ReturnFlagClient(client)
    ids = []

    def new_id():
        ids.append(random_goal_id())
        return ids[-1]
    r = NavGoalRunner(spy, wall_wait, time.monotonic, None, accept_timeout_s=0.5,
                      cancel_timeout_s=2.0, terminal_timeout_s=3.0, goal_id_fallback=canceller,
                      motion_stopped=lambda: True, new_goal_id=new_id)
    with InterruptAfterSend(spy, nth=1, mode='opcode') as inj:
        with pytest.raises(KeyboardInterrupt):
            r.run(goal(3), 'lost', 30.0)
    assert inj.where.startswith('run:')
    ours = ids[0]
    # the real server accepted and executes our goal although we hold no handle for it
    assert wait_until(lambda: canceller.status(ours) is not None, 5.0)
    ok = False
    end = time.monotonic() + 10.0
    while not ok and time.monotonic() < end:
        ok = r.cancel_all('shutdown')
    assert ok is True
    assert canceller.status(ours) == GoalStatus.STATUS_CANCELED
    assert not r.motion_inhibited()
    # the foreign goal is still executing: it was never cancelled
    time.sleep(0.3)
    assert canceller.status(foreign_id) == GoalStatus.STATUS_EXECUTING
    fres = fsend.result().get_result_async()
    assert not fres.done()
    fsend.result().cancel_goal_async()                      # clean up for the other tests
    assert wall_wait(fres, 3.0)


def test_unknown_id_is_reported_unknown_and_stays_unresolved(ros):
    _, client, _, canceller = ros
    never_sent = random_goal_id()
    fut = canceller.cancel(never_sent)
    assert wall_wait(fut, 3.0)
    assert fut.result().return_code == CancelGoal.Response.ERROR_UNKNOWN_GOAL_ID
    assert canceller.status(never_sent) is None


def test_terminal_status_is_published_per_goal_id(ros):
    _, client, _, canceller = ros
    gid = random_goal_id()
    send = client.send_goal_async(goal(1), goal_uuid=gid)    # succeeds after 0.3 s
    assert wall_wait(send, 3.0) and send.result().accepted
    assert wait_until(lambda: canceller.status(gid) == GoalStatus.STATUS_SUCCEEDED, 5.0)
    fut = canceller.cancel(gid)                              # cancelling a finished goal
    assert wall_wait(fut, 3.0)
    assert fut.result().return_code == CancelGoal.Response.ERROR_GOAL_TERMINATED


def test_cancel_all_form_is_refused(ros):
    _, _, _, canceller = ros
    with pytest.raises(ValueError):
        canceller.cancel(UUID(uuid=[0] * 16))
