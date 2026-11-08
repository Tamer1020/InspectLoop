"""ROS 2 gauge reading service.

Service  /gauge/read               (inspectloop_interfaces/srv/ReadGauge)
Sub      /camera/image_raw         (sensor_msgs/Image)
Sub      /camera/camera_info       (sensor_msgs/CameraInfo) -> intrinsics for solvePnP
Pub      /gauge/readings           (inspectloop_interfaces/msg/GaugeReading)
Pub      /gauge/annotated_image    (sensor_msgs/Image)

A request is served with a *burst* of N distinct frames whose header stamps are later than
the request time plus a settle delay, i.e. frames captured after the robot stopped at the
viewpoint. Each frame is read independently and the burst is fused into value +/- sigma
with quality flags (see gauge_reader.py). Missing frames within the timeout are reported
as NO_IMAGE / partial bursts instead of silently reusing old images.
"""
import json
import os
import threading
import time

import cv2
from cv_bridge import CvBridge
from inspectloop_interfaces.msg import GaugeReading
from inspectloop_interfaces.srv import ReadGauge
import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image

from inspectloop_vision import gauge_reader as gr


def _nan_to_num(x: float) -> float:
    return float('nan') if x is None else float(x)


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
        p('min_valid_frames', 3)
        p('min_visibility', 0.85)
        p('max_view_angle_deg', 55.0)
        p('min_dial_radius_px', 28.0)
        p('min_needle_confidence', 0.35)
        p('max_frame_spread', 0.20)
        p('min_sharpness', 15.0)
        p('sigma_floor', 0.03)

        self._bridge = CvBridge()
        self._reader = gr.GaugeReader()
        self._lock = threading.Lock()
        self._frames = []          # recent (stamp_ns, msg) ring buffer
        self._total_frames = 0
        self._K = None

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
    def _on_image(self, msg: Image):
        with self._lock:
            self._frames.append((Time.from_msg(msg.header.stamp).nanoseconds, msg))
            if len(self._frames) > 30:
                self._frames.pop(0)
            self._total_frames += 1

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
        """Return up to n frames with distinct stamps >= not_before (sim-time timeout)."""
        t0 = self.get_clock().now()
        wall_end = time.monotonic() + timeout_s * self.get_parameter('wall_timeout_factor').value
        while rclpy.ok():
            with self._lock:
                fresh = [m for s, m in self._frames if s >= not_before_ns]
            uniq = {}
            for m in fresh:
                uniq[Time.from_msg(m.header.stamp).nanoseconds] = m
            if len(uniq) >= n:
                return [uniq[k] for k in sorted(uniq)][:n]
            if ((self.get_clock().now() - t0).nanoseconds * 1e-9 > timeout_s or
                    time.monotonic() > wall_end):
                return [uniq[k] for k in sorted(uniq)]
            time.sleep(0.02)
        return []

    # ------------------------------------------------------------------ service
    def _on_read(self, req: ReadGauge.Request, resp: ReadGauge.Response):
        try:
            return self._read(req, resp)
        except Exception as e:  # noqa: BLE001 - the service must never kill the node
            self.get_logger().error(f'[{req.gauge_id}] gauge read failed: {e!r}')
            r = GaugeReading()
            r.gauge_id, r.tag_id, r.viewpoint_label = req.gauge_id, req.tag_id, req.viewpoint_label
            r.has_value, r.value, r.sigma = False, float('nan'), float('nan')
            r.quality_flags = ['NO_IMAGE']
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

        settle = Duration(seconds=float(self.get_parameter('settle_time_s').value))
        not_before = (self.get_clock().now() + settle).nanoseconds
        msgs = self._collect_burst(not_before, n, timeout + settle.nanoseconds * 1e-9)

        t0 = time.perf_counter()
        frames, images = [], []
        for m in msgs:
            bgr = self._bridge.imgmsg_to_cv2(m, desired_encoding='bgr8')
            images.append(bgr)
            frames.append(self._reader.read_frame(bgr, int(req.tag_id), self._K))
        obs = gr.fuse(frames, n, self.fuse_params())
        proc_ms = (time.perf_counter() - t0) * 1000.0

        r = GaugeReading()
        r.stamp = msgs[-1].header.stamp if msgs else self.get_clock().now().to_msg()
        r.gauge_id, r.tag_id, r.viewpoint_label = gid, int(req.tag_id), req.viewpoint_label
        r.has_value, r.value, r.sigma, r.unit = obs.has_value, obs.value, obs.sigma, gr.UNIT
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
        r.quality_flags = list(obs.flags)
        r.detail = obs.detail

        if images:
            # representative frame: the best-confidence frame with a tag, else the last one
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
        if obs.flags:
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


def reading_to_dict(r: GaugeReading) -> dict:
    return {
        'gauge_id': r.gauge_id, 'tag_id': r.tag_id, 'viewpoint': r.viewpoint_label,
        'stamp_s': round(r.stamp.sec + r.stamp.nanosec * 1e-9, 3),
        'has_value': r.has_value, 'value': _f(r.value), 'sigma': _f(r.sigma), 'unit': r.unit,
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
    rclpy.init(args=args)
    node = GaugeReaderNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
