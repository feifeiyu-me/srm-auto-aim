#!/usr/bin/env python3

import cv2

import rclpy
from rclpy.node import Node

from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class ImageSimNode(Node):

    def __init__(self):
        super().__init__('image_sim_node')

        self.bridge = CvBridge()

        self.image_path = (
            '/home/gyqros/srm_aim_ws/'
            'test_images/test_target.jpg'
        )

        self.image = cv2.imread(self.image_path)

        if self.image is None:
            self.get_logger().error(
                f'无法读取测试图片: {self.image_path}'
            )
            raise RuntimeError('Image load failed')

        height, width = self.image.shape[:2]

        self.get_logger().info(
            f'Loaded test image: '
            f'{width}x{height}'
        )

        self.publisher = self.create_publisher(
            Image,
            '/srm_aim/image_raw',
            10
        )

        # 模拟 30 FPS 摄像头
        self.timer = self.create_timer(
            1.0 / 30.0,
            self.timer_callback
        )

        self.frame_count = 0

        self.get_logger().info(
            'Image simulation node started.'
        )

        self.get_logger().info(
            'Publishing: /srm_aim/image_raw'
        )

    def timer_callback(self):

        msg = self.bridge.cv2_to_imgmsg(
            self.image,
            encoding='bgr8'
        )

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        msg.header.frame_id = 'camera'

        self.publisher.publish(msg)

        self.frame_count += 1

        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Published simulated frames: '
                f'{self.frame_count}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = ImageSimNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
