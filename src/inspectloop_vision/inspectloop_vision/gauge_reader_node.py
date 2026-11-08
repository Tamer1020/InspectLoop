"""ROS 2 gauge reading service.

Service  /gauge/read               (inspectloop_interfaces/srv/ReadGauge)
Sub      /camera/image_raw         (sensor_msgs/Image)
Sub      /camera/camera_info       (sensor_msgs/CameraInfo) -> intrinsics for solvePnP
Pub      /gauge/readings           (inspectloop_interfaces/msg/GaugeReading)
Pub      /gauge/annotated_image    (sensor_msgs/Image)

Freshness and provenance (Astra F5):
* A request is served with a *burst* of N distinct frames stamped in
  [request time + settle, now + max_future]: frames from before the request (another viewpoint)
  and implausibly future-dated frames are never used.
* A new `viewpoint_epoch` in the request invalidates every buffered frame.
* A backward jump of the node clock (simulation reset) clears the buffer; if it happens during
  a burst the observation is reported with CLOCK_RESET and no value.
* A burst whose frames are pixel-identical despite new stamps is FROZEN_FRAMES (stuck camera).
* The robot pose (map -> base_footprint) at each frame's capture time is looked up in TF.
  With check_pose, frames taken away from the expected viewpoint (WRONG_VIEWPOINT) or while the
  robot moved (ROBOT_MOVING) invalidate the observation.
* The reply echoes observation_id / viewpoint_epoch and carries the frame stamps, so the
  mission can re-validate it (observation_guard.py).
"""
from concurrent.futures import ProcessPoolExecutor
import json
import math
import multiprocessing
import os
import sys
import threading
import time
import zlib

import cv2
from cv_bridge import CvBridge
from inspectloop_interfaces.msg import GaugeReading
from inspectloop_interfaces.srv import ReadGauge
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.clock import JumpThreshold
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
import tf2_ros

from inspectloop_vision import gauge_reader as gr
from inspectloop_vision import reader_worker
from inspectloop_vision.frame_buffer import FrameBuffer, select_burst

INVALIDATING_FLAGS = ('FROZEN_FRAMES', 'CLOCK_RESET', 'WRONG_VIEWPOINT', 'ROBOT_MOVING')
# flags that mean the image data itself is not admissible evidence (F7 fault class IMAGE_INVALID)
IMAGE_INVALID_FLAGS = ('INVALID_FRAMES', 'FROZEN_FRAMES', 'CLOCK_RESET', 'WRONG_VIEWPOINT',
                       'ROBOT_MOVING')


def fault_class_for(has_value: bool, flags) -> str:
    """Typed outcome (V3, Astra V2 review F7)."""
    flags = set(flags)
    if 'PROCESSING_ERROR' in flags:
        return 'PROCESSING_ERROR'
    if 'NO_IMAGE' in flags:
        return 'CAMERA_UNAVAILABLE'
    if flags & set(IMAGE_INVALID_FLAGS):
        return 'IMAGE_INVALID'
    if has_value:
        return 'NONE'
    return 'INSUFFICIENT_EVIDENCE'


def _nan_to_num(x: float) -> float:
    return float('nan') if x is None else float(x)


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class GaugeReaderNode(Node):
    def __init__(self):
        super().__init__('gauge_reader')
        p = self.declare_parameter
        p('image_topic', '/camera/image_raw')
        p('camera_info_topic', '/camera/camera_info')
        p('default_output_dir', os.path.expanduser('~/inspectloop_runs/adhoc'))
        p('settle_time_s', 0.5)
        p('num_frames', 5)
        p('frame_timeout_s', 5.0)
        p('wall_timeout_factor', 3.0)
        p('max_frame_future_s', 0.2)
        p('clock_jump_tolerance_s', 0.001)   # V3: any larger backward step = clock reset
        p('max_pose_error_m', 0.35)
        p('max_pose_error_rad', 0.40)
        p('max_motion_m', 0.02)
        p('max_motion_rad', 0.02)
        p('min_valid_frames', 3)
        p('min_visibility', 0.85)
        p('max_view_angle_deg', 55.0)
        p('min_dial_radius_px', 40.0)
        p('min_needle_confidence', 0.35)
        p('max_frame_spread', 0.20)
        p('min_sharpness', 15.0)
        p('sigma_floor', 0.03)
        p('reader_worker_process', True)     # V3: read frames outside the node's GIL
        p('reader_worker_timeout_s', 20.0)

        self._bridge = CvBridge()
        self._reader = gr.GaugeReader()
        self._pool = None
        if self.get_parameter('reader_worker_process').value:
            self._start_pool()
        self._lock = threading.Lock()
        to_ns = lambda s: int(float(self.get_parameter(s).value) * 1e9)  # noqa: E731
        self._buf = FrameBuffer(maxlen=30, reset_threshold_ns=to_ns('clock_jump_tolerance_s'),
                                max_future_ns=to_ns('max_frame_future_s'))
        self._total_frames = 0
        self._clock_jumps = 0
        self._epoch = None
        # V3 (F5): every backward ROS-clock step (sim reset/replay), however small, is reported
        # by rcl; the buffer and any burst in progress are invalidated
        self._jump_handle = self.get_clock().create_jump_callback(
            JumpThreshold(min_forward=None, min_backward=Duration(nanoseconds=-1),
                          on_clock_change=True), post_callback=self._on_clock_jump)
        self._K = None
        self._tf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf, self)

        sensor_cb = MutuallyExclusiveCallbackGroup()
        self.create_subscription(Image, self.get_parameter('image_topic').value, self._on_image,
                                 qos_profile_sensor_data, callback_group=sensor_cb)
        self.create_subscription(CameraInfo, self.get_parameter('camera_info_topic').value,
                                 self._on_info, qos_profile_sensor_data, callback_group=sensor_cb)
        self._pub = self.create_publisher(GaugeReading, '/gauge/readings', 10)
        self._ann_pub = self.create_publisher(Image, '/gauge/annotated_image', 1)
        self.create_service(ReadGauge, '/gauge/read', self._on_read,
                            callback_group=ReentrantCallbackGroup())
        self.get_logger().info('gauge reading service ready on /gauge/read')

    # ------------------------------------------------------------------ subscriptions
    def _now_ns(self) -> int:
        return self.get_clock().now().nanoseconds

    def _on_image(self, msg: Image):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds
        digest = zlib.crc32(bytes(msg.data))
        with self._lock:
            if self._buf.add(stamp, digest, msg, self._now_ns()):
                self.get_logger().warning('camera/clock time jumped backwards: frame buffer cleared')
            self._total_frames += 1

    def _start_pool(self):
        ctx = multiprocessing.get_context('spawn')     # never fork a process with ROS threads
        self._pool = ProcessPoolExecutor(max_workers=1, mp_context=ctx)
        self._pool.submit(reader_worker.warm_up).result(timeout=60.0)

    def _read_frames(self, images, tag_id):
        """Per-frame reading, in the worker process if enabled. Raises on worker failure (the
        service then reports PROCESSING_ERROR) and restarts the worker for the next request."""
        if self._pool is None:
            return [self._reader.read_frame(img, int(tag_id), self._K) for img in images]
        try:
            fut = self._pool.submit(reader_worker.read_frames, images, int(tag_id), self._K)
            return fut.result(timeout=float(self.get_parameter('reader_worker_timeout_s').value))
        except Exception:
            self._pool.shutdown(wait=False, cancel_futures=True)
            self._start_pool()
            raise

    def destroy_node(self):
        if self._pool is not None:
            self._pool.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()

    def _on_clock_jump(self, jump):
        with self._lock:
            self._clock_jumps += 1
            self._buf.notify_clock_jump()
        self.get_logger().warning(
            f'ROS clock discontinuity ({jump.clock_change.name}, '
            f'{jump.delta.nanoseconds * 1e-9:+.3f} s): frame buffer cleared')

    def _on_info(self, msg: CameraInfo):
        if msg.k[0] > 0:
            self._K = np.array(msg.k, np.float64).reshape(3, 3)

    def fuse_params(self) -> gr.FuseParams:
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        return gr.FuseParams(int(g('min_valid_frames')), float(g('min_visibility')),
                             float(g('max_view_angle_deg')), float(g('min_dial_radius_px')),
                             float(g('min_needle_confidence')), float(g('max_frame_spread')),
                             float(g('min_sharpness')), float(g('sigma_floor')))

    def _collect_burst(self, not_before_ns: int, n: int, timeout_s: float):
        """Up to n fresh frames. Returns (messages, stats, clock_reset)."""
        t0 = self._now_ns()
        with self._lock:
            epoch0 = self._buf.time_epoch
            jumps0 = self._clock_jumps
        wall_end = time.monotonic() + timeout_s * self.get_parameter('wall_timeout_factor').value
        max_future = int(float(self.get_parameter('max_frame_future_s').value) * 1e9)
        stats = {}
        while rclpy.ok():
            now = self._now_ns()
            with self._lock:
                reset = (self._buf.check_clock(now) or self._buf.time_epoch != epoch0 or
                         self._clock_jumps != jumps0)
                sel, stats = select_burst(self._buf.frames(), not_before_ns, now, max_future, n)
            if reset:
                return [], stats, True
            if len(sel) >= n:
                return [f.payload for f in sel], stats, False
            if (now - t0) * 1e-9 > timeout_s or time.monotonic() > wall_end:
                return [f.payload for f in sel], stats, False
            time.sleep(0.02)
        return [], stats, False

    def _pose_at(self, stamp):
        try:
            tf = self._tf.lookup_transform('map', 'base_footprint', Time.from_msg(stamp),
                                           timeout=Duration(seconds=0.3))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return None
        t = tf.transform.translation
        return t.x, t.y, _yaw(tf.transform.rotation)

    # ------------------------------------------------------------------ service
    def _on_read(self, req: ReadGauge.Request, resp: ReadGauge.Response):
        try:
            return self._read(req, resp)
        except Exception as e:  # noqa: BLE001 - the service must never kill the node
            self.get_logger().error(f'[{req.gauge_id}] gauge read failed: {e!r}')
            r = GaugeReading()
            r.gauge_id, r.tag_id, r.viewpoint_label = req.gauge_id, req.tag_id, req.viewpoint_label
            r.observation_id, r.viewpoint_epoch = req.observation_id, req.viewpoint_epoch
            r.has_value, r.value, r.sigma = False, float('nan'), float('nan')
            # V3 (F7): a software fault, NOT a missing image / occlusion
            r.quality_flags = ['PROCESSING_ERROR']
            r.fault_class = 'PROCESSING_ERROR'
            r.detail = f'internal error: {type(e).__name__}: {e}'
            resp.success, resp.reading = False, r
            return resp

    def _read(self, req: ReadGauge.Request, resp: ReadGauge.Response):
        gid = req.gauge_id or 'gauge'
        n = int(req.num_frames) or int(self.get_parameter('num_frames').value)
        timeout = float(req.frame_timeout_s) or float(self.get_parameter('frame_timeout_s').value)
        out_dir = req.run_directory or self.get_parameter('default_output_dir').value
        img_dir = os.path.join(out_dir, 'images')
        os.makedirs(img_dir, exist_ok=True)

        request_time = self.get_clock().now()
        with self._lock:
            frames_before = self._total_frames
            jumps_before = self._clock_jumps
            if self._epoch != req.viewpoint_epoch:
                # new viewpoint: nothing captured before it may serve as evidence
                self._buf.invalidate()
                self._epoch = req.viewpoint_epoch
        settle = Duration(seconds=float(self.get_parameter('settle_time_s').value))
        not_before = (request_time + settle).nanoseconds
        msgs, stats, clock_reset = self._collect_burst(not_before, n,
                                                       timeout + settle.nanoseconds * 1e-9)

        # provenance FIRST: pose at capture time from TF, before the (slower) image processing,
        # while the frame stamps are certainly still inside the TF buffer
        poses = [self._pose_at(m.header.stamp) for m in msgs]

        t0 = time.perf_counter()
        images = [self._bridge.imgmsg_to_cv2(m, desired_encoding='bgr8') for m in msgs]
        frames = self._read_frames(images, req.tag_id) if images else []
        obs = gr.fuse(frames, n, self.fuse_params())
        proc_ms = (time.perf_counter() - t0) * 1000.0
        flags = list(obs.flags)

        r = GaugeReading()
        r.request_stamp = request_time.to_msg()
        r.stamp = msgs[-1].header.stamp if msgs else self.get_clock().now().to_msg()
        if msgs:
            r.first_frame_stamp = msgs[0].header.stamp

        # provenance: pose at capture time, motion during the burst
        r.pose_known = bool(poses) and all(p is not None for p in poses)
        if r.pose_known:
            x0, y0, a0 = poses[0]
            r.acq_x, r.acq_y, r.acq_yaw = (float(np.median([p[i] for p in poses])) for i in range(3))
            r.pose_motion_m = float(max(math.hypot(p[0] - x0, p[1] - y0) for p in poses))
            r.pose_motion_rad = float(max(abs(math.atan2(math.sin(p[2] - a0), math.cos(p[2] - a0)))
                                          for p in poses))
        else:
            r.acq_x = r.acq_y = r.acq_yaw = float('nan')
        with self._lock:
            frames_seen = self._total_frames - frames_before
            jumped = self._clock_jumps != jumps_before
        if not msgs and frames_seen > 0 and 'NO_IMAGE' in flags:
            # frames DID arrive but none was admissible (old / future-dated / clock reset)
            flags = [f for f in flags if f != 'NO_IMAGE'] + ['INVALID_FRAMES']
        if (clock_reset or jumped) and 'CLOCK_RESET' not in flags:
            flags.append('CLOCK_RESET')     # also a jump during processing invalidates the result
        if stats.get('frozen'):
            flags.append('FROZEN_FRAMES')
        if req.check_pose and msgs:
            tol_m = float(req.max_pose_error_m) or float(self.get_parameter('max_pose_error_m').value)
            tol_r = (float(req.max_pose_error_rad) or
                     float(self.get_parameter('max_pose_error_rad').value))
            if not r.pose_known:
                flags.append('WRONG_VIEWPOINT')
            else:
                d = math.hypot(r.acq_x - req.expected_x, r.acq_y - req.expected_y)
                da = abs(math.atan2(math.sin(r.acq_yaw - req.expected_yaw),
                                    math.cos(r.acq_yaw - req.expected_yaw)))
                if d > tol_m or da > tol_r:
                    flags.append('WRONG_VIEWPOINT')
                if (r.pose_motion_m > float(self.get_parameter('max_motion_m').value) or
                        r.pose_motion_rad > float(self.get_parameter('max_motion_rad').value)):
                    flags.append('ROBOT_MOVING')
        invalid = any(f in flags for f in INVALIDATING_FLAGS)

        r.gauge_id, r.tag_id, r.viewpoint_label = gid, int(req.tag_id), req.viewpoint_label
        r.observation_id, r.viewpoint_epoch = req.observation_id, req.viewpoint_epoch
        r.has_value = obs.has_value and not invalid
        r.value = obs.value if r.has_value else float('nan')
        r.sigma = obs.sigma if r.has_value else float('nan')
        r.unit = gr.UNIT
        r.frames_requested, r.frames_received = n, obs.frames_received
        r.frames_tag_detected, r.frames_valid = obs.frames_tag_detected, obs.frames_valid
        r.frame_values = [float(v) for v in obs.frame_values]
        r.frame_spread = float(obs.frame_spread)
        r.needle_confidence = float(obs.needle_confidence)
        r.dial_visibility = float(obs.dial_visibility)
        r.view_angle_deg = _nan_to_num(obs.view_angle_deg)
        r.distance_m = _nan_to_num(obs.distance_m)
        r.dial_radius_px = float(obs.dial_radius_px)
        r.sharpness = float(obs.sharpness)
        r.processing_time_ms = float(proc_ms)
        r.quality_flags = flags
        r.fault_class = fault_class_for(r.has_value, flags)
        r.detail = obs.detail
        if invalid:
            r.detail = 'observation invalidated: ' + ', '.join(
                f for f in flags if f in INVALIDATING_FLAGS) + ' | ' + obs.detail
        if stats.get('future'):
            r.detail += f" ({stats['future']} future-dated frame(s) rejected)"

        if images:
            idx = max(range(len(frames)), key=lambda i: (frames[i].tag_found,
                                                         frames[i].needle_confidence))
            stamp_s = Time.from_msg(msgs[idx].header.stamp).nanoseconds * 1e-9
            vp = (req.viewpoint_label.split()[0] if req.viewpoint_label else 'V')
            base = os.path.join(img_dir, f'{gid}_{vp}_t{stamp_s:08.2f}')
            title = f'{gid} {req.viewpoint_label}'
            ann = gr.annotate(images[idx], frames[idx], obs, title)
            cv2.imwrite(base + '_raw.png', images[idx])
            cv2.imwrite(base + '_annotated.png', ann)
            r.raw_image_path, r.annotated_image_path = base + '_raw.png', base + '_annotated.png'
            if frames[idx].homography is not None:
                cv2.imwrite(base + '_rectified.png',
                            gr.GaugeReader.rectify(images[idx], frames[idx].homography))
                r.rectified_image_path = base + '_rectified.png'
            out = self._bridge.cv2_to_imgmsg(ann, encoding='bgr8')
            out.header = msgs[idx].header
            self._ann_pub.publish(out)
        else:
            with self._lock:
                total = self._total_frames
            r.detail += f' (no fresh frame within {timeout:.1f}s; frames received so far: {total})'

        summary = f'[{gid}] {req.viewpoint_label}: {r.detail}'
        if flags:
            self.get_logger().warning(summary)
        else:
            self.get_logger().info(summary)
        self._pub.publish(r)
        with open(os.path.join(out_dir, 'gauge_readings.jsonl'), 'a') as f:
            f.write(json.dumps(reading_to_dict(r)) + '\n')
        resp.success = bool(msgs)
        resp.reading = r
        return resp


def _f(x, nd=4):
    x = float(x)
    return None if x != x else round(x, nd)   # NaN -> null in JSON


def _t(stamp) -> float:
    return round(stamp.sec + stamp.nanosec * 1e-9, 3)


def reading_to_dict(r: GaugeReading) -> dict:
    return {
        'gauge_id': r.gauge_id, 'tag_id': r.tag_id, 'viewpoint': r.viewpoint_label,
        'observation_id': r.observation_id, 'viewpoint_epoch': r.viewpoint_epoch,
        'fault_class': r.fault_class, 'stamp_s': _t(r.stamp), 'request_stamp_s': _t(r.request_stamp),
        'first_frame_stamp_s': _t(r.first_frame_stamp), 'last_frame_stamp_s': _t(r.stamp),
        'pose_known': r.pose_known, 'acq_x': _f(r.acq_x, 3), 'acq_y': _f(r.acq_y, 3),
        'acq_yaw': _f(r.acq_yaw, 3), 'pose_motion_m': _f(r.pose_motion_m, 4),
        'pose_motion_rad': _f(r.pose_motion_rad, 4),
        'has_value': r.has_value, 'value': _f(r.value), 'sigma_heuristic': _f(r.sigma),
        'unit': r.unit,
        'frames_requested': r.frames_requested, 'frames_received': r.frames_received,
        'frames_tag_detected': r.frames_tag_detected, 'frames_valid': r.frames_valid,
        'frame_values': [_f(v) for v in r.frame_values], 'frame_spread': _f(r.frame_spread),
        'needle_confidence': _f(r.needle_confidence, 3), 'dial_visibility': _f(r.dial_visibility, 3),
        'view_angle_deg': _f(r.view_angle_deg, 1), 'distance_m': _f(r.distance_m, 3),
        'dial_radius_px': _f(r.dial_radius_px, 1), 'sharpness': _f(r.sharpness, 1),
        'processing_time_ms': _f(r.processing_time_ms, 1), 'quality_flags': list(r.quality_flags),
        'raw_image_path': r.raw_image_path, 'annotated_image_path': r.annotated_image_path,
        'rectified_image_path': r.rectified_image_path, 'detail': r.detail,
    }


def main(args=None):
    # shorter GIL switch interval for the work that stays in this process (V3, see reader_worker)
    sys.setswitchinterval(0.001)
    rclpy.init(args=args)
    node = GaugeReaderNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:   # noqa: BLE001 - context invalidated by a concurrent shutdown (SIGINT race)
        if rclpy.ok():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
