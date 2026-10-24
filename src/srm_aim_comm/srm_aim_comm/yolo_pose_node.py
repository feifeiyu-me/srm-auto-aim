"""Independent YOLO11 pose inference node; never publishes control commands."""

from pathlib import Path
import time

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from srm_aim_interfaces.msg import ArmorDetectionArray
from srm_aim_comm.yolo_pose_adapter import result_to_message, validate_model


class YoloPoseNode(Node):
    """Process received images with a local, explicitly configured armor model."""

    def __init__(self):
        super().__init__('yolo_pose_node')
        try:
            defaults = {
                'model_path': '~/srm_aim_ws/models/best.pt',
                'image_topic': '/srm_aim/image_raw',
                'detections_topic': '/srm_aim/armor_detections',
                'confidence_threshold': 0.25,
                'keypoint_confidence_threshold': 0.5,
                'corner_indices': [0, 1, 2, 3],
                'image_size': 640,
                'device': 'cpu',
                'max_detections': 100,
                'min_quad_area': 16.0,
                'min_edge_length': 2.0,
                'max_edge_ratio': 20.0,
                'warn_inference_sec': 0.1,
            }
            values = {
                name: self.declare_parameter(
                    name, value, ParameterDescriptor(read_only=True)).value
                for name, value in defaults.items()
            }
            self._configure(values)
            path = Path(values['model_path']).expanduser()
            if not path.is_file() or path.suffix.lower() != '.pt':
                raise ValueError(f'Model must be an existing local .pt file: {path}')
            # Import only after the explicit local path check; no default model.
            from ultralytics import YOLO
            self.model = YOLO(str(path))
            validate_model(self.model)
            self.bridge = CvBridge()
            self.publisher = self.create_publisher(
                ArmorDetectionArray, values['detections_topic'], 1)
            self.subscription = self.create_subscription(
                Image, values['image_topic'], self.image_callback,
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
            self.get_logger().info(
                f'YOLO pose ready: {path}; device={self.device}; '
                f'publishing {values["detections_topic"]}')
        except Exception as exc:
            self.get_logger().error(f'YOLO startup failed; inference disabled: {exc}')
            self.destroy_node()
            raise RuntimeError(f'YOLO startup failed: {exc}') from exc

    def _configure(self, values):
        self.conversion = {
            'confidence': float(values['confidence_threshold']),
            'keypoint_confidence': float(values['keypoint_confidence_threshold']),
            'corner_indices': list(values['corner_indices']),
            'min_area': float(values['min_quad_area']),
            'min_edge': float(values['min_edge_length']),
            'max_edge_ratio': float(values['max_edge_ratio']),
        }
        for key in ('confidence', 'keypoint_confidence'):
            value = self.conversion[key]
            if not np.isfinite(value) or not 0 <= value <= 1:
                raise ValueError(f'{key} must be within [0, 1]')
        if sorted(self.conversion['corner_indices']) != [0, 1, 2, 3]:
            raise ValueError('corner_indices must be a permutation of [0,1,2,3]')
        for key in ('min_area', 'min_edge', 'max_edge_ratio'):
            value = self.conversion[key]
            if not np.isfinite(value) or value <= 0:
                raise ValueError(f'{key} must be finite and positive')
        if self.conversion['max_edge_ratio'] < 1:
            raise ValueError('max_edge_ratio must be at least 1')
        self.image_size = int(values['image_size'])
        self.max_detections = int(values['max_detections'])
        self.device = values['device']
        self.warn_inference_sec = float(values['warn_inference_sec'])
        if (self.image_size <= 0 or self.max_detections <= 0
                or not np.isfinite(self.warn_inference_sec)
                or self.warn_inference_sec <= 0):
            raise ValueError('Image size, max detections and delay threshold must be positive')

    def image_callback(self, image):
        started = time.monotonic()
        try:
            if image.width <= 0 or image.height <= 0 or not len(image.data):
                raise ValueError('Empty image')
            if image.encoding not in ('bgr8', 'rgb8', 'mono8', 'bgra8', 'rgba8'):
                raise ValueError(f'Unsupported image encoding: {image.encoding}')
            frame = self.bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
            if (frame.dtype != np.uint8
                    or frame.shape != (image.height, image.width, 3)):
                raise ValueError('Decoded image shape/type does not match its metadata')
            results = self.model.predict(
                source=frame, conf=self.conversion['confidence'],
                imgsz=self.image_size, device=self.device,
                max_det=self.max_detections, verbose=False)
            if len(results) != 1:
                raise ValueError('Expected exactly one result for one input image')
            message = result_to_message(
                results[0], image.header, image.width, image.height,
                **self.conversion)
            self.publisher.publish(message)
            rejected = len(results[0].boxes.conf) - len(message.detections)
            if rejected:
                self.get_logger().debug(f'Filtered {rejected} invalid/low-confidence poses')
        except Exception as exc:
            self.get_logger().error(
                f'YOLO image/inference failed; frame dropped: {exc}',
                throttle_duration_sec=5.0)
        finally:
            elapsed = time.monotonic() - started
            if elapsed > self.warn_inference_sec:
                self.get_logger().warning(
                    f'YOLO frame processing took {elapsed:.3f}s; '
                    'source timestamp is preserved; input frames may be dropped under load',
                    throttle_duration_sec=5.0)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = YoloPoseNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
