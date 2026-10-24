"""Independent, exact-frame YOLO visualization with bounded receipt-time caches."""

import os
import sys
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from srm_aim_interfaces.msg import ArmorDetectionArray
from srm_aim_comm.yolo_visualization import (
    DetectionRate, ExactFrameCache, draw_frame, frame_key,
)


class YoloVisualizerNode(Node):
    """Only display verified image/detection pairs; no inference or control I/O."""

    def __init__(self):
        super().__init__('yolo_visualizer_node')
        self._window_open = False
        self.stop_requested = False
        try:
            defaults = {
                'image_topic': '/srm_aim/image_raw',
                'detections_topic': '/srm_aim/armor_detections',
                'image_cache_size': 20,
                'detection_cache_size': 20,
                'max_cache_bytes': 96 * 1024 * 1024,
                'cache_ttl_sec': 1.0,
                'display_timeout_sec': 0.5,
                'refresh_hz': 30.0,
                'window_name': 'SRM YOLO Pose',
                'window_width': 1280,
                'window_height': 960,
            }
            p = {name: self.declare_parameter(
                name, value, ParameterDescriptor(read_only=True)).value
                for name, value in defaults.items()}
            for key in ('refresh_hz', 'display_timeout_sec'):
                if not np.isfinite(p[key]) or p[key] <= 0:
                    raise ValueError(f'{key} must be finite and positive')
            if min(p['window_width'], p['window_height']) <= 0 or not p['window_name']:
                raise ValueError('Window dimensions and name must be nonempty/positive')
            self.cache = ExactFrameCache(p['image_cache_size'], p['detection_cache_size'],
                                         p['max_cache_bytes'], p['cache_ttl_sec'])
            self.rate = DetectionRate()
            self.bridge = CvBridge()
            self.window = p['window_name']
            self.display_timeout = p['display_timeout_sec']
            self._ready = None
            self._last_image = None
            self._last_detection = None
            self._last_match_time = None
            self._latest_stamp = None
            if sys.platform.startswith('linux') and not (
                    os.environ.get('DISPLAY') or os.environ.get('WAYLAND_DISPLAY')):
                raise RuntimeError('OpenCV display unavailable: set DISPLAY or run under Xvfb')
            cv2.namedWindow(self.window, cv2.WINDOW_NORMAL)
            self._window_open = True
            cv2.resizeWindow(self.window, p['window_width'], p['window_height'])
            self.image_sub = self.create_subscription(
                Image, p['image_topic'], self.image_callback,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            # Best Effort reader accepts both Reliable and Best Effort writers.
            self.detection_sub = self.create_subscription(
                ArmorDetectionArray, p['detections_topic'], self.detection_callback,
                QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.timer = self.create_timer(1.0 / p['refresh_hz'], self.update_display)
            self.get_logger().info('Visualizer ready; exact Header matching; press q to exit.')
        except Exception as exc:
            self.get_logger().error(f'Visualizer startup failed: {exc}')
            self.destroy_node()
            raise

    def _queue(self, pair, now):
        if pair is None:
            return
        stamp = frame_key(pair[0].header)[:2]
        if self._latest_stamp is not None and stamp <= self._latest_stamp:
            return
        self._latest_stamp = stamp
        self._ready = pair[0], pair[1], now

    def image_callback(self, image):
        try:
            if image.encoding not in ('bgr8', 'rgb8', 'mono8', 'bgra8', 'rgba8'):
                raise ValueError(f'Unsupported image encoding: {image.encoding}')
            now = time.monotonic()
            self._queue(self.cache.add_image(image, now), now)
        except Exception as exc:
            self.get_logger().warning(f'Image not cached: {exc}', throttle_duration_sec=5.0)

    def detection_callback(self, detection):
        now = time.monotonic()
        self.rate.add(now)
        try:
            self._queue(self.cache.add_detection(detection, now), now)
        except Exception as exc:
            self.get_logger().warning(f'Detection not matched: {exc}', throttle_duration_sec=5.0)

    def update_display(self):
        now = time.monotonic()
        self.cache.prune(now)
        if self._ready is not None:
            image, detection, arrived = self._ready
            self._ready = None
            if now - arrived < self.cache.ttl:
                try:
                    frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
                    if frame.shape != (image.height, image.width, 3):
                        raise ValueError('Decoded image dimensions differ from Header metadata')
                    self._last_image = frame
                    self._last_detection = detection
                    self._last_match_time = arrived
                except Exception as exc:
                    self.get_logger().warning(f'Matched image decode failed: {exc}',
                                              throttle_duration_sec=5.0)
        if self._last_image is None:
            canvas = np.zeros((240, 640, 3), dtype=np.uint8)
            cv2.putText(canvas, 'Waiting for exact image/detection match', (12, 100),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.65, (255, 255, 255), 1)
        else:
            age = now - self._last_match_time
            canvas = draw_frame(self._last_image, self._last_detection,
                                self.rate.fps(now), age, stale=age > self.display_timeout)
        cv2.imshow(self.window, canvas)
        if (cv2.waitKey(1) & 0xff == ord('q')
                or cv2.getWindowProperty(self.window, cv2.WND_PROP_VISIBLE) == 0):
            self.stop_requested = True

    def destroy_node(self):
        if self._window_open:
            self._window_open = False
            try:
                cv2.destroyWindow(self.window)
            except cv2.error:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloVisualizerNode()
        while rclpy.ok() and not node.stop_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
