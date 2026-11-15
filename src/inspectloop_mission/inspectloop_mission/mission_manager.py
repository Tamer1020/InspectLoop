"""InspectLoop mission manager: uncertainty-aware autonomous gauge inspection.

For every gauge in the mission file:

    vp = planner.initial()                     # frontal viewpoint at nominal standoff
    loop:
        navigate to vp (Nav2 NavigateToPose, retries + timeouts)
        observe: /gauge/read  (burst of fresh frames -> value, sigma, quality flags)
        decision = policy.decide(observation, retries at this pose, viewpoints left, budget)
            ACCEPT      -> record value, heuristic sigma and alarm state; next gauge
            RETRY_SAME  -> wait, observe again from the same pose
            MOVE_NEW    -> planner.next(hint) -> navigate there (Nav2) -> observe
            UNRESOLVED  -> record honestly why no reliable reading exists; next gauge

Every observation and decision is published (/gauge/readings, /mission/decisions), drawn in
RViz (/mission/markers) and stored in mission_record.json, from which the JSON / Markdown /
HTML reports are rendered.

Safety / failure handling (kept from the navigation layer):
  * navigation goal rejected/aborted/timeout -> retry, then treat the viewpoint as unreachable
  * LiDAR stream ERROR in /diagnostics       -> cancel goal and ABORT the mission
    (cmd_vel_gate independently holds the robot stopped); the report is still written
  * system not ready within the timeout      -> ABORT before moving
"""
import datetime
import json
import math
import os
import signal
import threading
import time
import traceback
from typing import Dict, List, Optional
import uuid

from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from inspectloop_interfaces.msg import GaugeDecision, MissionStatus
from inspectloop_interfaces.srv import ReadGauge
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from std_srvs.srv import SetBool
from rclpy.time import Time
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from inspectloop_mission import decision_policy as dp
from inspectloop_mission.mission_config import alarm_state, GaugeSpec, load_mission, Pose2D
from inspectloop_mission.nav_goal import GoalIdCanceller, NavGoalRunner
from inspectloop_mission.observation_guard import (ExpectedObservation, is_settled,
                                                   validate_observation)
from inspectloop_mission.report import write_reports
from inspectloop_mission.viewpoint_planner import OccupancyMap, Viewpoint, ViewpointPlanner

# diagnostic_msgs levels are 'byte' fields (python bytes); compare as ints
LVL_OK, LVL_WARN, LVL_ERROR, LVL_STALE = 0, 1, 2, 3
LEVELS = {LVL_OK: 'OK', LVL_WARN: 'WARN', LVL_ERROR: 'ERROR', LVL_STALE: 'STALE'}
DECISION_RGB = {dp.ACCEPT: (0.1, 0.85, 0.2), dp.RETRY_SAME: (1.0, 0.85, 0.0),
                dp.MOVE_NEW: (1.0, 0.5, 0.0), dp.UNRESOLVED: (0.9, 0.1, 0.1)}


def level_to_int(level) -> int:
    if isinstance(level, (bytes, bytearray)):
        return int.from_bytes(level, 'little')
    return int(level)


def yaw_to_quat(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def quat_to_yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def angle_diff(a: float, b: float) -> float:
    return math.atan2(math.sin(a - b), math.cos(a - b))


def _num(x, nd=4):
    if x is None:
        return None
    x = float(x)
    return None if x != x else round(x, nd)


class MissionAborted(Exception):
    pass


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')
        p = self.declare_parameter
        p('mission_file', '')
        p('output_root', os.path.expanduser('~/inspectloop_runs'))
        p('nav_timeout_s', 180.0)
        p('nav_attempts_per_viewpoint', 2)
        p('retry_backoff_s', 2.0)
        p('retry_same_delay_s', 1.0)
        p('frames_per_observation', 5)
        p('frame_timeout_s', 5.0)
        p('max_sigma', 0.15)
        p('min_valid_frames', 3)
        p('max_retries_same', 1)
        p('max_viewpoints', 3)
        p('max_observations', 8)
        p('viewpoint_distances', [1.5, 1.2, 1.9])
        p('viewpoint_angles_deg', [0.0, -30.0, 30.0, -45.0, 45.0])
        p('viewpoint_clearance_m', 0.45)
        p('system_ready_timeout_s', 180.0)
        p('return_home', True)
        p('abort_on_lidar_failure', True)
        p('lidar_diagnostic_name', 'inspectloop/stream/scan')
        p('wall_timeout_factor', 4.0)
        p('settle_window_s', 0.5)            # robot at rest this long before measuring (F5)
        p('settle_timeout_s', 5.0)
        p('settle_max_v', 0.02)
        p('settle_max_w', 0.05)
        p('nav_accept_timeout_s', 15.0)      # F4 goal lifecycle bounds
        p('nav_cancel_timeout_s', 10.0)
        p('nav_terminal_timeout_s', 15.0)
        p('nav_inhibit_grace_s', 30.0)       # wait for an unresolved goal before e-stop + abort
        p('gate_fault_abort_s', 2.0)         # gate STOPPED_SCAN_FAULT this long -> abort (F3)
        p('gate_unobserved_abort_s', 3.0)    # gate BLOCKED_UNOBSERVED this long -> abort (V3 F3)
        p('max_result_age_s', 8.0)           # V3 F5: last frame -> consumption, sim seconds
        p('odom_max_age_s', 0.3)             # V3 F5: settle check needs fresh odometry

        self._cb = ReentrantCallbackGroup()
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self._cb)
        self._read = self.create_client(ReadGauge, '/gauge/read', callback_group=self._cb)
        self._clear = [self.create_client(ClearEntireCostmap, s, callback_group=self._cb) for s in (
            '/global_costmap/clear_entirely_global_costmap',
            '/local_costmap/clear_entirely_local_costmap')]
        self._estop = self.create_client(SetBool, '/cmd_vel_gate/set_estop', callback_group=self._cb)
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                             reliability=ReliabilityPolicy.RELIABLE)
        self._status_pub = self.create_publisher(MissionStatus, '/mission/status', 10)
        self._decision_pub = self.create_publisher(GaugeDecision, '/mission/decisions', 50)
        self._marker_pub = self.create_publisher(MarkerArray, '/mission/markers', latched)
        self.create_subscription(DiagnosticArray, '/diagnostics', self._on_diag, 50,
                                 callback_group=self._cb)
        # AMCL and map_server latch their outputs (transient local)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, latched,
                                 callback_group=self._cb)
        self.create_subscription(OccupancyGrid, '/map', self._on_map, latched,
                                 callback_group=self._cb)
        self.create_subscription(Odometry, '/odom', self._on_odom, qos_profile_sensor_data,
                                 callback_group=self._cb)
        self.create_subscription(String, '/cmd_vel_gate/state', self._on_gate, latched,
                                 callback_group=self._cb)

        self._lock = threading.Lock()
        self._diag_levels: Dict[str, int] = {}
        self._health_events: List[Dict] = []
        self._amcl_received = False
        self._map: Optional[OccupancyMap] = None
        self._feedback = None
        self._markers: Dict[tuple, Marker] = {}
        self._odom = []                       # (t, |v|, |w|) recent samples
        self._gate_state = None
        self._gate_fault_since = None
        self._gate_unobserved_since = None
        self._epoch = 0                       # viewpoint epoch (F5)
        self._clock_epoch = 0                 # V3 F5: increments on every ROS-clock discontinuity
        self._clock_jumps: List[Dict] = []
        self._jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1),
                          on_clock_change=True), post_callback=self._on_clock_jump)
        self.mission_id = ''
        # V3.2 F4: unknown-ownership goals are cancelled by THEIR goal id only (never the
        # cancel-all form: RViz / waypoint_follower share navigate_to_pose) and confirmed via
        # the server's status topic plus a stationary robot
        self._nav_by_id = GoalIdCanceller(self, 'navigate_to_pose', callback_group=self._cb)
        self.nav = NavGoalRunner(
            self._nav, self._wait_future, self._sim_now, self.get_logger(),
            accept_timeout_s=float(self.get_parameter('nav_accept_timeout_s').value),
            cancel_timeout_s=float(self.get_parameter('nav_cancel_timeout_s').value),
            terminal_timeout_s=float(self.get_parameter('nav_terminal_timeout_s').value),
            goal_id_fallback=self._nav_by_id, motion_stopped=self._robot_stopped)
        self.status = MissionStatus()
        self.status.state = 'IDLE'
        self.create_timer(0.5, self._publish_status, callback_group=self._cb)

    # ------------------------------------------------------------------ callbacks
    def _on_diag(self, msg: DiagnosticArray):
        t = self._sim_now()
        with self._lock:
            for st in msg.status:
                if not st.name.startswith('inspectloop/'):
                    continue
                lvl = level_to_int(st.level)
                prev = self._diag_levels.get(st.name)
                if prev != lvl:
                    self._diag_levels[st.name] = lvl
                    if prev is not None or lvl != LVL_OK:
                        self._health_events.append({'t': round(t, 2), 'name': st.name,
                                                    'level': LEVELS.get(lvl, str(lvl)),
                                                    'message': st.message})

    def _on_amcl(self, _msg):
        self._amcl_received = True

    def _on_clock_jump(self, jump):
        with self._lock:
            self._clock_epoch += 1
            self._odom.clear()
            self._clock_jumps.append({'wall': datetime.datetime.now().isoformat(timespec='seconds'),
                                      'change': jump.clock_change.name,
                                      'delta_s': round(jump.delta.nanoseconds * 1e-9, 3),
                                      'clock_epoch': self._clock_epoch})
        self.get_logger().warning(f'ROS clock discontinuity ({jump.clock_change.name}, '
                                  f'{jump.delta.nanoseconds * 1e-9:+.3f} s): clock epoch '
                                  f'{self._clock_epoch}, pending observations invalidated')

    def clock_epoch(self) -> int:
        with self._lock:
            return self._clock_epoch

    def _on_odom(self, msg: Odometry):
        # V3 F5: the message stamp (acquisition time), not the receive time
        t = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9
        v = math.hypot(msg.twist.twist.linear.x, msg.twist.twist.linear.y)
        with self._lock:
            if self._odom and t < self._odom[-1][0]:
                self._odom.clear()                       # out of order / clock reset
            self._odom.append((t, v, abs(msg.twist.twist.angular.z)))
            if len(self._odom) > 200:
                del self._odom[:100]

    def _on_gate(self, msg: String):
        with self._lock:
            self._gate_state = msg.data
            if msg.data == 'STOPPED_SCAN_FAULT':
                self._gate_fault_since = self._gate_fault_since or time.monotonic()
            else:
                self._gate_fault_since = None
            if msg.data == 'BLOCKED_UNOBSERVED':
                self._gate_unobserved_since = self._gate_unobserved_since or time.monotonic()
            else:
                self._gate_unobserved_since = None

    def _on_map(self, msg: OccupancyGrid):
        data = np.array(msg.data, np.int16).reshape(msg.info.height, msg.info.width)
        self._map = OccupancyMap(data, msg.info.resolution, msg.info.origin.position.x,
                                 msg.info.origin.position.y)

    def _publish_status(self):
        self.status.stamp = self.get_clock().now().to_msg()
        self._status_pub.publish(self.status)

    # ------------------------------------------------------------------ helpers
    def _sim_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _set(self, state: str, detail: str = ''):
        self.status.state = state
        self.status.detail = detail
        self._publish_status()

    def _wait_future(self, fut, timeout_s: float, abort_check=None):
        """Wait for an rclpy future using sim time, with a wall-clock backstop."""
        t0 = self._sim_now()
        wall_end = time.monotonic() + timeout_s * self.get_parameter('wall_timeout_factor').value
        while rclpy.ok() and not fut.done():
            if self._sim_now() - t0 > timeout_s or time.monotonic() > wall_end:
                return False
            if abort_check is not None:
                abort_check()
            time.sleep(0.05)
        return fut.done()

    def _sleep_sim(self, seconds: float):
        t0 = self._sim_now()
        wall_end = time.monotonic() + seconds * 4 + 1.0
        while rclpy.ok() and self._sim_now() - t0 < seconds and time.monotonic() < wall_end:
            time.sleep(0.05)

    def _robot_stopped(self) -> bool:
        """Fresh odometry shows the robot stationary (same thresholds as observation)."""
        P = self.get_parameter
        with self._lock:
            samples = list(self._odom)
        return is_settled(samples, float(P('settle_window_s').value),
                          float(P('settle_max_v').value), float(P('settle_max_w').value),
                          now_s=self._sim_now(),
                          max_age_s=float(P('odom_max_age_s').value))

    def lidar_level(self) -> Optional[int]:
        with self._lock:
            return self._diag_levels.get(self.get_parameter('lidar_diagnostic_name').value)

    def _check_lidar(self):
        if not self.get_parameter('abort_on_lidar_failure').value:
            return
        if self.lidar_level() == LVL_ERROR:
            raise MissionAborted('LiDAR stream failed (health_monitor ERROR) - stopping for safety')
        with self._lock:
            since = self._gate_fault_since
            unobserved = self._gate_unobserved_since
        if since is not None and time.monotonic() - since > float(
                self.get_parameter('gate_fault_abort_s').value):
            # scans may still arrive but be unusable (empty / NaN / bad stamps): the gate knows
            raise MissionAborted('LiDAR data invalid (cmd_vel_gate STOPPED_SCAN_FAULT) - '
                                 'stopping for safety')
        if unobserved is not None and time.monotonic() - unobserved > float(
                self.get_parameter('gate_unobserved_abort_s').value):
            # V3 F3: the scan is structurally fine but the commanded direction is not observed
            raise MissionAborted('LiDAR does not observe the motion corridor (cmd_vel_gate '
                                 'BLOCKED_UNOBSERVED) - stopping for safety')

    def latch_estop(self, reason: str) -> bool:
        """Hard motion inhibition through the safety gate (used when a goal can't be confirmed
        cancelled). The operator must release it explicitly."""
        self.get_logger().error(f'latching e-stop: {reason}')
        if not self._estop.wait_for_service(timeout_sec=2.0):
            return False
        fut = self._estop.call_async(SetBool.Request(data=True))
        return self._wait_future(fut, 5.0) and fut.result() is not None and fut.result().success

    def robot_pose(self) -> Optional[Pose2D]:
        try:
            tf = self._tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        t = tf.transform.translation
        return Pose2D(t.x, t.y, quat_to_yaw(tf.transform.rotation))

    def _clear_costmaps(self):
        for c in self._clear:
            if c.service_is_ready():
                self._wait_future(c.call_async(ClearEntireCostmap.Request()), 5.0)

    # ------------------------------------------------------------------ readiness
    def wait_for_system(self) -> None:
        self._set('WAITING_FOR_SYSTEM', 'waiting for Nav2, gauge reader, map and localization')
        timeout = self.get_parameter('system_ready_timeout_s').value
        t_wall = time.monotonic()
        missing = []
        while rclpy.ok() and time.monotonic() - t_wall < timeout:
            missing = []
            if not self._nav.server_is_ready():
                missing.append('navigate_to_pose action')
            if not self._read.service_is_ready():
                missing.append('/gauge/read service')
            if self._map is None:
                missing.append('/map')
            if self.robot_pose() is None:
                missing.append('TF map->base_footprint')
            if not self._amcl_received:
                missing.append('/amcl_pose')
            if not missing:
                self.get_logger().info('system ready')
                return
            time.sleep(0.5)
        raise MissionAborted('system not ready after %.0fs: missing %s' % (timeout, ', '.join(missing)))

    # ------------------------------------------------------------------ navigation
    def navigate(self, name: str, pose: Pose2D, attempts: int) -> Dict:
        timeout = float(self.get_parameter('nav_timeout_s').value)
        rec = {'outcome': 'FAILED', 'attempts': [], 'total_time_s': 0.0}
        t_start = self._sim_now()
        for attempt in range(1, attempts + 1):
            self._check_lidar()
            self._set('NAVIGATING', f'{name} attempt {attempt}/{attempts}')
            a = self._nav_attempt(name, pose, timeout)
            if a['result'] == 'INHIBITED':
                self._resolve_or_inhibit(name)
                a = self._nav_attempt(name, pose, timeout)
            a['attempt'] = attempt
            rec['attempts'].append(a)
            if a['result'] == 'SUCCEEDED':
                rec['outcome'] = 'SUCCEEDED'
                break
            self.get_logger().warning(
                f'[{name}] nav attempt {attempt} failed: {a["result"]} {a.get("detail", "")}')
            if attempt < attempts:
                self._clear_costmaps()
                self._sleep_sim(float(self.get_parameter('retry_backoff_s').value))
        rec['total_time_s'] = round(self._sim_now() - t_start, 2)
        final = self.robot_pose()
        if final is not None:
            rec['final_pose'] = {'x': round(final.x, 3), 'y': round(final.y, 3),
                                 'yaw': round(final.yaw, 3)}
            rec['position_error_m'] = round(math.hypot(final.x - pose.x, final.y - pose.y), 3)
            rec['yaw_error_rad'] = round(abs(angle_diff(final.yaw, pose.yaw)), 3)
        return rec

    def _nav_attempt(self, name: str, pose: Pose2D, timeout: float) -> Dict:
        self._feedback = None
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x, goal.pose.pose.position.y = pose.x, pose.y
        o = goal.pose.pose.orientation
        o.x, o.y, o.z, o.w = yaw_to_quat(pose.yaw)

        def on_feedback(fb):
            self._feedback = fb.feedback

        out = self.nav.run(goal, name, timeout, abort_check=self._check_lidar,
                           feedback_cb=on_feedback)
        fb = self._feedback
        out['recoveries'] = int(fb.number_of_recoveries) if fb is not None else 0
        return out

    def _resolve_or_inhibit(self, name: str):
        """A previous goal is not confirmed terminated: give it a bounded grace period, else
        latch the gate e-stop and abort (never start new motion over an orphan goal)."""
        grace = float(self.get_parameter('nav_inhibit_grace_s').value)
        t0 = time.monotonic()
        while time.monotonic() - t0 < grace:
            if self.nav.cancel_all(f'resolve before {name}'):
                return
            time.sleep(0.5)
        unresolved = ', '.join(self.nav.unresolved())
        self.latch_estop(f'unresolved navigation goal(s): {unresolved}')
        raise MissionAborted(f'navigation goal(s) {unresolved} could not be confirmed cancelled '
                             '- motion inhibited (gate e-stop latched)')

    # ------------------------------------------------------------------ observation
    def wait_settled(self) -> bool:
        P = self.get_parameter
        t0 = time.monotonic()
        while time.monotonic() - t0 < float(P('settle_timeout_s').value) * 2.0:
            with self._lock:
                samples = list(self._odom)
            if is_settled(samples, float(P('settle_window_s').value), float(P('settle_max_v').value),
                          float(P('settle_max_w').value), now_s=self._sim_now(),
                          max_age_s=float(P('odom_max_age_s').value)):
                return True
            time.sleep(0.05)
        return False

    def observe(self, g: GaugeSpec, vp: Viewpoint, run_dir: str, obs_index: int) -> Dict:
        self._set('OBSERVING', f'{g.id} @ {vp.label}')
        settled = self.wait_settled()
        req = ReadGauge.Request()
        req.gauge_id, req.tag_id, req.viewpoint_label = g.id, g.tag_id, vp.label
        req.num_frames = int(self.get_parameter('frames_per_observation').value)
        req.frame_timeout_s = float(self.get_parameter('frame_timeout_s').value)
        req.run_directory = run_dir
        req.observation_id = f'{self.mission_id}/{g.id}/{vp.label.split()[0]}/{obs_index}'
        req.viewpoint_epoch = self._epoch
        req.check_pose = True
        req.expected_x, req.expected_y, req.expected_yaw = vp.x, vp.y, vp.yaw
        # V3 (F7): a service problem is a SYSTEM fault, never reported as a missing image
        fail = {'viewpoint': vp.label, 'observation_id': req.observation_id,
                'mission_id': self.mission_id, 'viewpoint_epoch': self._epoch,
                'has_value': False, 'value': None, 'sigma': None, 'frames_valid': 0,
                'quality_flags': ['SERVICE_FAILURE'], 'fault_class': 'SERVICE_FAILURE',
                'settled_before_request': settled}
        if not self._read.wait_for_service(timeout_sec=5.0):
            return dict(fail, detail='gauge reading service unavailable')
        request_time = self._sim_now()
        clock_epoch_request = self.clock_epoch()
        fut = self._read.call_async(req)
        if not self._wait_future(fut, req.frame_timeout_s + 20.0):
            # abandoned: a late reply carries this observation_id and will never be consumed
            return dict(fail, detail='gauge reading service call timed out')
        try:
            resp = fut.result()
        except Exception as e:  # noqa: BLE001 - service call raised on the client side
            return dict(fail, detail=f'gauge reading service call failed: {e!r}')
        if resp is None:
            return dict(fail, detail='gauge reading service returned no response')
        r = resp.reading
        t = lambda st: round(st.sec + st.nanosec * 1e-9, 3)  # noqa: E731
        out = {
            'viewpoint': vp.label, 'observation_id': r.observation_id,
            'mission_id': self.mission_id, 'fault_class': r.fault_class or None,
            'viewpoint_epoch': r.viewpoint_epoch, 'settled_before_request': settled,
            'request_time_s': round(request_time, 3),
            'first_frame_stamp_s': t(r.first_frame_stamp), 'last_frame_stamp_s': t(r.stamp),
            'stamp_s': t(r.stamp), 'pose_known': r.pose_known,
            'acq_x': _num(r.acq_x, 3), 'acq_y': _num(r.acq_y, 3), 'acq_yaw': _num(r.acq_yaw, 3),
            'pose_motion_m': _num(r.pose_motion_m, 4), 'pose_motion_rad': _num(r.pose_motion_rad, 4),
            'has_value': r.has_value, 'value': _num(r.value), 'sigma': _num(r.sigma),
            'unit': r.unit, 'frames_requested': r.frames_requested,
            'frames_received': r.frames_received, 'frames_tag_detected': r.frames_tag_detected,
            'frames_valid': r.frames_valid, 'frame_values': [_num(v) for v in r.frame_values],
            'frame_spread': _num(r.frame_spread), 'needle_confidence': _num(r.needle_confidence, 3),
            'dial_visibility': _num(r.dial_visibility, 3),
            'view_angle_deg': _num(r.view_angle_deg, 1), 'distance_m': _num(r.distance_m, 3),
            'dial_radius_px': _num(r.dial_radius_px, 1), 'sharpness': _num(r.sharpness, 1),
            'processing_time_ms': _num(r.processing_time_ms, 1),
            'quality_flags': list(r.quality_flags), 'raw_image_path': r.raw_image_path,
            'annotated_image_path': r.annotated_image_path,
            'rectified_image_path': r.rectified_image_path, 'detail': r.detail,
        }
        # re-validate before the result may become evidence for THIS viewpoint (F5)
        consumed = self._sim_now()
        clock_epoch_now = self.clock_epoch()
        exp = ExpectedObservation(req.observation_id, self._epoch, request_time, consumed,
                                  vp.x, vp.y, vp.yaw,
                                  max_result_age_s=float(self.get_parameter('max_result_age_s').value),
                                  clock_epoch_request=clock_epoch_request,
                                  clock_epoch_now=clock_epoch_now, mission_id=self.mission_id)
        out['consumed_time_s'] = round(consumed, 3)
        last = out.get('last_frame_stamp_s')
        out['result_age_s'] = (round(consumed - last, 3)
                               if isinstance(last, float) and out.get('frames_received') else None)
        out['clock_epoch_request'], out['clock_epoch_consumed'] = clock_epoch_request, clock_epoch_now
        problems = validate_observation(out, exp)
        if not settled:
            problems.append('robot not settled before the request')
        if problems:
            out['provenance_problems'] = problems
            out['quality_flags'] = list(out['quality_flags']) + ['STALE_OBSERVATION']
            out['has_value'], out['value'], out['sigma'] = False, None, None
            out['fault_class'] = 'IMAGE_INVALID'
            out['detail'] = 'rejected by mission: ' + '; '.join(problems)
        return out

    def _publish_decision(self, g: GaugeSpec, idx: int, vp_label: str, d: dp.Decision, obs: Dict):
        m = GaugeDecision()
        m.stamp = self.get_clock().now().to_msg()
        m.gauge_id, m.observation_index, m.viewpoint_label = g.id, idx, vp_label
        m.decision, m.decision_label, m.reason = d.code, d.decision, d.reason
        m.has_value = bool(obs.get('has_value'))
        m.value = float('nan') if obs.get('value') is None else float(obs['value'])
        m.sigma = float('nan') if obs.get('sigma') is None else float(obs['sigma'])
        m.quality_flags = list(obs.get('quality_flags') or [])
        self._decision_pub.publish(m)
        self.status.last_decision = f'{g.id}#{idx} {d.decision}'

    # ------------------------------------------------------------------ one gauge
    def inspect_gauge(self, g: GaugeSpec, run_dir: str, gi: int) -> Dict:
        P = self.get_parameter
        policy = dp.PolicyParams(float(P('max_sigma').value), int(P('min_valid_frames').value),
                                 int(P('max_retries_same').value), int(P('max_observations').value))
        max_vps = int(P('max_viewpoints').value)
        planner = ViewpointPlanner(
            g.x, g.y, g.normal_yaw, self._map, distances=list(P('viewpoint_distances').value),
            angles_deg=list(P('viewpoint_angles_deg').value),
            clearance_m=float(P('viewpoint_clearance_m').value))
        rec = {'id': g.id, 'tag_id': g.tag_id, 'description': g.description, 'unit': g.unit,
               'mount': {'x': g.x, 'y': g.y, 'normal_yaw': g.normal_yaw},
               'alarm_low': g.alarm_low, 'alarm_high': g.alarm_high,
               # V3 F8: footprint-clear on the map only - Nav2 reachability is NOT pre-checked
               'candidates_feasible': [v.as_dict() for v in planner.candidates],
               'candidate_feasibility': 'footprint-clear in the occupancy map; Nav2 path '
                                        'reachability not pre-checked',
               'candidates_rejected': [v.as_dict() for v in planner.rejected],
               'viewpoints': [], 'timeline': [], 'result': None}
        self._marker_candidates(gi, planner)
        t0 = self._sim_now()
        vp = planner.initial()
        obs_count = 0
        final = None
        while final is None:
            if vp is None:
                final = {'status': 'UNRESOLVED',
                         'reason': 'no viewpoint candidate left (all footprint-clear candidates '
                                   'tried, blocked or found unreachable)'}
                break
            planner.mark_tried(vp)
            self._epoch += 1                       # new viewpoint epoch (F5)
            self.status.current_viewpoint = vp.label
            self.get_logger().info(f'[{g.id}] -> viewpoint {vp.label} ({vp.x:.2f}, {vp.y:.2f})')
            nav = self.navigate(f'{g.id}/{vp.label}', Pose2D(vp.x, vp.y, vp.yaw),
                                int(P('nav_attempts_per_viewpoint').value))
            vrec = {'viewpoint': vp.as_dict(), 'navigation': nav, 'observations': []}
            rec['viewpoints'].append(vrec)
            vps_left = min(max_vps - len(rec['viewpoints']), len(planner.available()))
            self._marker_viewpoint(gi, len(rec['viewpoints']), vp, None)
            if nav['outcome'] != 'SUCCEEDED':
                last = nav['attempts'][-1]['result'] if nav['attempts'] else 'FAILED'
                if vps_left > 0:
                    d = dp.Decision(dp.MOVE_NEW, f'viewpoint unreachable ({last})')
                else:
                    d = dp.Decision(dp.UNRESOLVED, 'viewpoint unreachable and no viewpoint left')
                rec['timeline'].append({'t': round(self._sim_now() - t0, 1), 'viewpoint': vp.label,
                                        'event': 'NAV_FAILED', 'decision': d.decision,
                                        'reason': d.reason})
                self._publish_decision(g, obs_count, vp.label, d, {})
                self._marker_viewpoint(gi, len(rec['viewpoints']), vp, d.decision)
                if d.decision == dp.UNRESOLVED:
                    final = {'status': 'UNRESOLVED', 'reason': d.reason}
                else:
                    vp = planner.next('any')
                continue
            retries = 0
            while True:
                obs_count += 1
                self.status.current_observation = obs_count
                obs = self.observe(g, vp, run_dir, obs_count)
                d = dp.decide(obs, retries, vps_left, obs_count, policy)
                obs['decision'] = d.decision
                obs['reason'] = d.reason
                obs['index'] = obs_count
                vrec['observations'].append(obs)
                rec['timeline'].append({
                    't': round(self._sim_now() - t0, 1), 'viewpoint': vp.label,
                    'event': f'OBSERVATION #{obs_count}', 'value': obs.get('value'),
                    'sigma': obs.get('sigma'), 'flags': obs.get('quality_flags'),
                    'decision': d.decision, 'reason': d.reason})
                self._publish_decision(g, obs_count, vp.label, d, obs)
                self._marker_viewpoint(gi, len(rec['viewpoints']), vp, d.decision)
                val = obs.get('value')
                shown = f'{val:.2f} +/- {obs["sigma"]:.2f}' if val is not None else 'no value'
                self.get_logger().info(
                    f'[{g.id}] obs #{obs_count} @ {vp.label}: {shown} '
                    f'flags={obs.get("quality_flags")} -> {d.decision} ({d.reason})')
                self._write_partial_gauge(rec, run_dir)
                if d.decision == dp.ACCEPT:
                    # +-2 sigma of a HEURISTIC uncertainty model: a diagnostic band, not a
                    # statistically validated confidence interval (Astra F6)
                    final = {'status': 'ACCEPTED', 'value': obs['value'],
                             'sigma_heuristic': obs['sigma'],
                             'band_2sigma_heuristic': [round(obs['value'] - 2 * obs['sigma'], 3),
                                                       round(obs['value'] + 2 * obs['sigma'], 3)],
                             'alarm': alarm_state(g, obs['value']), 'viewpoint': vp.label,
                             'annotated_image_path': obs.get('annotated_image_path'),
                             'reason': d.reason}
                    break
                if d.decision == dp.RETRY_SAME:
                    retries += 1
                    self._sleep_sim(float(P('retry_same_delay_s').value))
                    continue
                if d.decision == dp.MOVE_NEW:
                    if d.hint == 'different_side':
                        planner.mark_blocked_direction(vp)
                    vp = planner.next(d.hint, reference=vp)
                    rec['timeline'][-1]['hint'] = d.hint
                    break
                final = {'status': 'UNRESOLVED', 'reason': d.reason}
                break
        last_obs = next((o for v in reversed(rec['viewpoints'])
                         for o in reversed(v['observations'])), None)
        nav_failed = bool(rec['timeline']) and rec['timeline'][-1].get('event') == 'NAV_FAILED'
        final['outcome_class'] = dp.outcome_class(
            final['status'], (last_obs or {}).get('quality_flags') or (), nav_failed)
        final['observations'] = obs_count
        final['viewpoints_used'] = len(rec['viewpoints'])
        final['time_s'] = round(self._sim_now() - t0, 1)
        rec['result'] = final
        self._marker_result(gi, g, final)
        return rec

    # ------------------------------------------------------------------ RViz markers
    def _marker(self, ns, mid, mtype, x, y, rgb, scale, text='', yaw=0.0, z=0.05, alpha=0.9):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns, m.id, m.type = ns, mid, mtype
        m.pose.position.x, m.pose.position.y, m.pose.position.z = x, y, z
        o = m.pose.orientation
        o.x, o.y, o.z, o.w = yaw_to_quat(yaw)
        m.scale.x, m.scale.y, m.scale.z = scale
        m.color.r, m.color.g, m.color.b = rgb
        m.color.a = alpha
        m.text = text
        self._markers[(ns, mid)] = m
        self._marker_pub.publish(MarkerArray(markers=list(self._markers.values())))

    def _marker_candidates(self, gi, planner):
        for v in planner.candidates:
            self._marker(f'candidates_g{gi}', v.index, Marker.ARROW, v.x, v.y, (0.5, 0.5, 0.8),
                         (0.3, 0.04, 0.04), yaw=v.yaw, alpha=0.5)

    def _marker_viewpoint(self, gi, n, vp, decision):
        rgb = DECISION_RGB.get(decision, (0.3, 0.6, 1.0))
        self._marker(f'visited_g{gi}', n, Marker.ARROW, vp.x, vp.y, rgb, (0.5, 0.08, 0.08),
                     yaw=vp.yaw, z=0.1, alpha=1.0)

    def _marker_result(self, gi, g, final):
        if final['status'] == 'ACCEPTED':
            text = (f"{g.id}: {final['value']:.2f} {g.unit} (sigma~{final['sigma_heuristic']:.2f}) "
                    f"[{final['alarm']}]")
            rgb = (0.1, 0.85, 0.2) if final['alarm'] == 'NORMAL' else (1.0, 0.5, 0.0)
        else:
            text = f'{g.id}: UNRESOLVED'
            rgb = (0.9, 0.1, 0.1)
        ox, oy = math.cos(g.normal_yaw) * 0.4, math.sin(g.normal_yaw) * 0.4
        self._marker('results', gi, Marker.TEXT_VIEW_FACING, g.x + ox, g.y + oy, rgb,
                     (0.0, 0.0, 0.3), text=text, z=1.2, alpha=1.0)

    # ------------------------------------------------------------------ main flow
    def _write_partial_gauge(self, rec, run_dir):
        with open(os.path.join(run_dir, f'gauge_{rec["id"]}.json'), 'w') as f:
            json.dump(rec, f, indent=2)

    def _write_record(self, record, run_dir):
        with open(os.path.join(run_dir, 'mission_record.json'), 'w') as f:
            json.dump(record, f, indent=2)

    def run(self) -> Dict:
        mission_file = self.get_parameter('mission_file').value
        cfg = load_mission(mission_file)
        mission_id = datetime.datetime.now().strftime('%Y%m%d_%H%M%S') + '_' + uuid.uuid4().hex[:4]
        run_dir = os.path.join(self.get_parameter('output_root').value, mission_id)
        os.makedirs(os.path.join(run_dir, 'images'), exist_ok=True)
        self.status.mission_id = mission_id
        self.mission_id = mission_id
        self.status.gauges_total = len(cfg.gauges)
        P = self.get_parameter
        record = {
            'mission_id': mission_id, 'mission_name': cfg.name, 'mission_file': mission_file,
            'run_dir': run_dir, 'state': 'RUNNING', 'abort_reason': None,
            'started_wall': datetime.datetime.now().isoformat(timespec='seconds'),
            'parameters': {k: P(k).value for k in (
                'max_sigma', 'min_valid_frames', 'max_retries_same', 'max_viewpoints',
                'max_observations', 'frames_per_observation', 'viewpoint_distances',
                'viewpoint_angles_deg', 'viewpoint_clearance_m', 'nav_timeout_s',
                'nav_attempts_per_viewpoint', 'max_result_age_s', 'odom_max_age_s')},
            'gauges': [], 'home_return': None,
        }
        self.get_logger().info(
            f'mission {mission_id} "{cfg.name}": {len(cfg.gauges)} gauges -> {run_dir}')
        t_wall0 = time.monotonic()
        t_sim0 = None
        try:
            self.wait_for_system()
            t_sim0 = self._sim_now()
            for gi, g in enumerate(cfg.gauges):
                self.status.current_gauge = g.id
                self.get_logger().info(f'=== {g.id} (tag {g.tag_id}) {g.description}')
                grec = self.inspect_gauge(g, run_dir, gi)
                record['gauges'].append(grec)
                res = grec['result']
                self.status.gauges_done = gi + 1
                self.status.gauges_accepted += int(res['status'] == 'ACCEPTED')
                self.status.gauges_unresolved += int(res['status'] != 'ACCEPTED')
                if res['status'] == 'ACCEPTED':
                    self.get_logger().info(
                        f'[{g.id}] ACCEPTED {res["value"]:.2f} {g.unit} (heuristic sigma '
                        f'{res["sigma_heuristic"]:.2f}) alarm={res["alarm"]} after {res["observations"]} obs / '
                        f'{res["viewpoints_used"]} viewpoint(s)')
                else:
                    self.get_logger().warning(f'[{g.id}] UNRESOLVED: {res["reason"]}')
                self._write_record(record, run_dir)
            if cfg.home is not None and self.get_parameter('return_home').value:
                self.status.current_gauge = 'HOME'
                self._set('RETURNING', 'returning to home pose')
                record['home_return'] = self.navigate('HOME', cfg.home, 2)
            record['state'] = 'COMPLETED'
        except MissionAborted as e:
            record['state'] = 'ABORTED'
            record['abort_reason'] = str(e)
            self.get_logger().error(f'MISSION ABORTED: {e}')
        except KeyboardInterrupt:
            record['state'] = 'ABORTED'
            record['abort_reason'] = 'interrupted (SIGINT)'
            self.get_logger().error('MISSION INTERRUPTED')
        except Exception as e:  # noqa: BLE001 - always produce a report
            record['state'] = 'ABORTED'
            record['abort_reason'] = f'internal error: {type(e).__name__}: {e}'
            self.get_logger().error(record['abort_reason'] + '\n' + traceback.format_exc())
        # never leave a goal running behind us (F4): cancel, confirm, else latch the e-stop.
        # V3: cancel_all() never raises; if anything in this block still fails (incl. a second
        # Ctrl-C) the e-stop is latched anyway - cleanup is never reported as successful.
        cleanup_ok = False
        try:
            cleanup_ok = self.nav.cancel_all('mission end')
        except BaseException as e:  # noqa: BLE001
            self.get_logger().error(f'goal cleanup raised: {e!r}')
            record['cleanup_exception'] = f'{type(e).__name__}: {e}'
        record['nav_cleanup_confirmed'] = cleanup_ok
        if not cleanup_ok:
            try:
                record['estop_latched'] = self.latch_estop(
                    'navigation goal not confirmed cancelled at mission end')
            except BaseException as e:  # noqa: BLE001
                record['estop_latched'] = False
                self.get_logger().error(f'latching the e-stop failed: {e!r}')
            record['unresolved_goals'] = self.nav.unresolved()
            record['abort_reason'] = (record.get('abort_reason') or '') + \
                ' | unresolved navigation goal at mission end: e-stop ' + \
                ('latched' if record['estop_latched'] else 'NOT confirmed - check the robot')
        record['duration_wall_s'] = round(time.monotonic() - t_wall0, 1)
        record['duration_sim_s'] = round(self._sim_now() - t_sim0, 1) if t_sim0 else 0.0
        with self._lock:
            record['health_events'] = list(self._health_events)
            record['clock_discontinuities'] = list(self._clock_jumps)
        record['nav_goal_events'] = list(self.nav.events)
        record['uncertainty_note'] = ('sigma is a HEURISTIC model (frame spread + pixel resolution '
                                      '+ floor); bands are diagnostic, not calibrated confidence '
                                      'intervals')
        self._write_record(record, run_dir)
        paths = write_reports(record, run_dir)
        self._set(record['state'], record.get('abort_reason') or 'report: ' + paths['markdown'])
        self.get_logger().info(f'mission {record["state"]}. Report: {paths["markdown"]}')
        return record


def _raise_interrupt(signum, _frame):
    raise KeyboardInterrupt(f'signal {signum}')


def install_interrupt_handlers():
    """SIGINT/SIGTERM -> KeyboardInterrupt in the mission (main) thread, explicitly.

    Python only installs its default SIGINT handler if SIGINT was not ignored at start-up, and
    processes started as background jobs of a non-interactive shell inherit SIGINT=SIG_IGN
    (found in the Gazebo interrupt test: SIGINT was silently ignored and the robot kept driving).
    SIGTERM is handled too because ros2 launch escalates to it."""
    signal.signal(signal.SIGINT, _raise_interrupt)
    signal.signal(signal.SIGTERM, _raise_interrupt)


def main(args=None):
    # no rclpy signal handler: SIGINT raises KeyboardInterrupt in the mission thread while the
    # executor keeps spinning, so active goals can still be cancelled and confirmed (F4)
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    install_interrupt_handlers()
    node = MissionManager()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    spin = threading.Thread(target=executor.spin, daemon=True)
    spin.start()
    rc = 0
    try:
        record = node.run()
        rc = 0 if record['state'] == 'COMPLETED' else 2
    except (KeyboardInterrupt, ExternalShutdownException):
        rc = 130
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == '__main__':
    raise SystemExit(main())
