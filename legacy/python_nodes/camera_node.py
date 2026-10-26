#!/usr/bin/env python3

import time

import cv2
import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CameraNode(Node):

    def __init__(self):
        super().__init__('camera_node')

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            Image,
            '/srm_aim/image_raw',
            10
        )

        # =========================
        # 摄像头配置
        # =========================
        self.device = '/dev/video0'
        self.width = 1280
        self.height = 720
        self.fps = 30

        self.cap = cv2.VideoCapture(
            self.device,
            cv2.CAP_V4L2
        )

        if not self.cap.isOpened():
            self.get_logger().error(
                f'无法打开摄像头 {self.device}'
            )
            raise RuntimeError('Camera open failed')

        # MJPG
        self.cap.set(
            cv2.CAP_PROP_FOURCC,
            cv2.VideoWriter_fourcc(*'MJPG')
        )

        # 分辨率
        self.cap.set(
            cv2.CAP_PROP_FRAME_WIDTH,
            self.width
        )

        self.cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT,
            self.height
        )

        # 帧率
        self.cap.set(
            cv2.CAP_PROP_FPS,
            self.fps
        )

        # 读取实际配置
        actual_width = self.cap.get(
            cv2.CAP_PROP_FRAME_WIDTH
        )

        actual_height = self.cap.get(
            cv2.CAP_PROP_FRAME_HEIGHT
        )

        actual_fps = self.cap.get(
            cv2.CAP_PROP_FPS
        )

        self.get_logger().info(
            f'Camera opened: '
            f'{actual_width:.0f}x{actual_height:.0f}, '
            f'MJPG, requested FPS={self.fps}, '
            f'camera reports FPS={actual_fps:.1f}'
        )

        # =========================
        # 状态统计
        # =========================
        self.frame_count = 0
        self.fail_count = 0

        self.last_report_time = time.monotonic()
        self.last_report_frame = 0

        # ROS2 定时器
        self.timer = self.create_timer(
            1.0 / self.fps,
            self.timer_callback
        )

        self.get_logger().info(
            'Camera node started.'
        )

        self.get_logger().info(
            'Publishing: /srm_aim/image_raw'
        )

    def timer_callback(self):

        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.fail_count += 1

            self.get_logger().warning(
                f'摄像头读取失败，'
                f'连续失败次数: {self.fail_count}'
            )

            return

        # 成功读取
        self.fail_count = 0

        # =========================
        # OpenCV BGR
        # →
        # ROS2 sensor_msgs/Image
        # =========================
        msg = self.bridge.cv2_to_imgmsg(
            frame,
            encoding='bgr8'
        )

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'camera'

        self.publisher.publish(msg)

        self.frame_count += 1

        # =========================
        # 每秒统计一次实际发布 FPS
        # =========================
        now = time.monotonic()

        elapsed = now - self.last_report_time

        if elapsed >= 1.0:

            frames = (
                self.frame_count
                - self.last_report_frame
            )

            actual_publish_fps = frames / elapsed

            self.get_logger().info(
                f'Published frames: '
                f'{self.frame_count}, '
                f'publish FPS: '
                f'{actual_publish_fps:.1f}'
            )

            self.last_report_time = now
            self.last_report_frame = self.frame_count

    def destroy_node(self):

        self.get_logger().info(
            'Stopping camera...'
        )

        if self.cap is not None:
            self.cap.release()

        super().destroy_node()


def main(args=None):

    rclpy.init(args=args)

    node = None

    try:
        node = CameraNode()

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    except Exception as e:
        print(f'Camera node error: {e}')

    finally:

        if node is not None:
            node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':
    main()
