"""InspectLoop mission manager: uncertainty-aware autonomous gauge inspection.

For every gauge in the mission file:

    vp = planner.initial()                     # frontal viewpoint at nominal standoff
    loop:
        navigate to vp (Nav2 NavigateToPose, retries + timeouts)
        observe: /gauge/read  (burst of fresh frames -> value, sigma, quality flags)
        decision = policy.decide(observation, retries at this pose, viewpoints left, budget)
            ACCEPT      -> record value +/- 2 sigma and alarm state; next gauge
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
import threading
import time
import traceback
from typing import Dict, List, Optional
import uuid

from action_msgs.msg import GoalStatus
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from inspectloop_interfaces.msg import GaugeDecision, MissionStatus
from inspectloop_interfaces.srv import ReadGauge
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from inspectloop_mission import decision_policy as dp
from inspectloop_mission.mission_config import alarm_state, GaugeSpec, load_mission, Pose2D
from inspectloop_mission.report import write_reports
from inspectloop_mission.viewpoint_planner import OccupancyMap, Viewpoint, ViewpointPlanner

GOAL_STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
               GoalStatus.STATUS_CANCELED: 'CANCELED', GoalStatus.STATUS_UNKNOWN: 'UNKNOWN'}
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

        self._cb = ReentrantCallbackGroup()
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose', callback_group=self._cb)
        self._read = self.create_client(ReadGauge, '/gauge/read', callback_group=self._cb)
        self._clear = [self.create_client(ClearEntireCostmap, s, callback_group=self._cb) for s in (
            '/global_costmap/clear_entirely_global_costmap',
            '/local_costmap/clear_entirely_local_costmap')]
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

        self._lock = threading.Lock()
        self._diag_levels: Dict[str, int] = {}
        self._health_events: List[Dict] = []
        self._amcl_received = False
        self._map: Optional[OccupancyMap] = None
        self._feedback = None
        self._markers: Dict[tuple, Marker] = {}
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

    def lidar_level(self) -> Optional[int]:
        with self._lock:
            return self._diag_levels.get(self.get_parameter('lidar_diagnostic_name').value)

    def _check_lidar(self):
        if self.get_parameter('abort_on_lidar_failure').value and self.lidar_level() == LVL_ERROR:
            raise MissionAborted('LiDAR stream failed (health_monitor ERROR) - stopping for safety')

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
        t0 = self._sim_now()
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

        send = self._nav.send_goal_async(goal, feedback_callback=on_feedback)
        if not self._wait_future(send, 15.0):
            return {'result': 'NO_RESPONSE', 'duration_s': round(self._sim_now() - t0, 2),
                    'detail': 'action server did not answer the goal request'}
        handle = send.result()
        if handle is None or not handle.accepted:
            return {'result': 'REJECTED', 'duration_s': round(self._sim_now() - t0, 2),
                    'detail': 'goal rejected by bt_navigator'}
        result_fut = handle.get_result_async()
        try:
            done = self._wait_future(result_fut, timeout, abort_check=self._check_lidar)
        except MissionAborted:
            self._wait_future(handle.cancel_goal_async(), 5.0)
            raise
        fb = self._feedback
        recoveries = int(fb.number_of_recoveries) if fb is not None else 0
        if not done:
            self.get_logger().error(f'[{name}] navigation timeout after {timeout:.0f}s - canceling')
            self._wait_future(handle.cancel_goal_async(), 5.0)
            return {'result': 'TIMEOUT', 'duration_s': round(self._sim_now() - t0, 2),
                    'recoveries': recoveries, 'detail': f'no result within {timeout:.0f}s'}
        res = result_fut.result()
        status = GOAL_STATUS.get(res.status, f'STATUS_{res.status}')
        out = {'result': status, 'duration_s': round(self._sim_now() - t0, 2),
               'recoveries': recoveries}
        if status != 'SUCCEEDED':
            err = getattr(res.result, 'error_code', 0)
            msg = getattr(res.result, 'error_msg', '')
            out['detail'] = f'nav2 error_code={err} {msg}'.strip()
        return out

    # ------------------------------------------------------------------ observation
    def observe(self, g: GaugeSpec, vp: Viewpoint, run_dir: str) -> Dict:
        self._set('OBSERVING', f'{g.id} @ {vp.label}')
        req = ReadGauge.Request()
        req.gauge_id, req.tag_id, req.viewpoint_label = g.id, g.tag_id, vp.label
        req.num_frames = int(self.get_parameter('frames_per_observation').value)
        req.frame_timeout_s = float(self.get_parameter('frame_timeout_s').value)
        req.run_directory = run_dir
        fail = {'viewpoint': vp.label, 'has_value': False, 'value': None, 'sigma': None,
                'frames_valid': 0, 'quality_flags': ['NO_IMAGE']}
        if not self._read.wait_for_service(timeout_sec=5.0):
            return dict(fail, detail='gauge reading service unavailable')
        fut = self._read.call_async(req)
        if not self._wait_future(fut, req.frame_timeout_s + 20.0):
            return dict(fail, detail='gauge reading service call timed out')
        r = fut.result().reading
        pose = self.robot_pose()
        return {
            'viewpoint': vp.label, 'stamp_s': round(r.stamp.sec + r.stamp.nanosec * 1e-9, 3),
            'robot_pose': pose.as_dict() if pose else None,
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
               'candidates_feasible': [v.as_dict() for v in planner.candidates],
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
                         'reason': 'no reachable viewpoint left (all candidates tried or blocked)'}
                break
            planner.mark_tried(vp)
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
                obs = self.observe(g, vp, run_dir)
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
                    final = {'status': 'ACCEPTED', 'value': obs['value'], 'sigma': obs['sigma'],
                             'ci95': [round(obs['value'] - 2 * obs['sigma'], 3),
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
            text = (f"{g.id}: {final['value']:.2f} +/- {2 * final['sigma']:.2f} {g.unit} "
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
                'nav_attempts_per_viewpoint')},
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
                        f'[{g.id}] ACCEPTED {res["value"]:.2f} +/- {2 * res["sigma"]:.2f} '
                        f'{g.unit} (95%) alarm={res["alarm"]} after {res["observations"]} obs / '
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
        except Exception as e:  # noqa: BLE001 - always produce a report
            record['state'] = 'ABORTED'
            record['abort_reason'] = f'internal error: {type(e).__name__}: {e}'
            self.get_logger().error(record['abort_reason'] + '\n' + traceback.format_exc())
        record['duration_wall_s'] = round(time.monotonic() - t_wall0, 1)
        record['duration_sim_s'] = round(self._sim_now() - t_sim0, 1) if t_sim0 else 0.0
        with self._lock:
            record['health_events'] = list(self._health_events)
        self._write_record(record, run_dir)
        paths = write_reports(record, run_dir)
        self._set(record['state'], record.get('abort_reason') or 'report: ' + paths['markdown'])
        self.get_logger().info(f'mission {record["state"]}. Report: {paths["markdown"]}')
        return record


def main(args=None):
    rclpy.init(args=args)
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
