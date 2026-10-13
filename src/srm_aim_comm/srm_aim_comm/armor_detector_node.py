import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped


class ArmorDetectorNode(Node):
    def __init__(self):
        super().__init__('armor_detector_node')

        self.bridge = CvBridge()
        self.frame_count = 0

        self.subscription = self.create_subscription(
            Image, '/srm_aim/image_raw', self.image_callback, 10
        )
        self.target_pub = self.create_publisher(
            PointStamped, '/srm_aim/target', 10
        )

        self.get_logger().info('Armor detector with visualization started.')

    def find_light_bars(self, mask):
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        bars = []

        for contour in contours:
            area = cv2.contourArea(contour)
            if area < 20:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if h < 10:
                continue

            ratio = h / max(w, 1)
            if ratio < 1.5:
                continue

            bars.append({
                'contour': contour,
                'x': x, 'y': y, 'w': w, 'h': h,
                'area': area,
                'aspect_ratio': ratio,
                'center_x': x + w / 2,
                'center_y': y + h / 2
            })

        return bars

    def match_light_bars(self, bars):
        pairs = []

        for i in range(len(bars)):
            for j in range(i + 1, len(bars)):
                left, right = bars[i], bars[j]

                if left['center_x'] > right['center_x']:
                    left, right = right, left

                height_ratio = min(left['h'], right['h']) / max(
                    left['h'], right['h']
                )
                if height_ratio < 0.7:
                    continue

                max_height = max(left['h'], right['h'])
                if abs(left['center_y'] - right['center_y']) > max_height * 0.3:
                    continue

                distance = right['center_x'] - left['center_x']
                if not max_height * 0.3 <= distance <= max_height * 3.0:
                    continue

                pairs.append({
                    'left': left,
                    'right': right,
                    'center_x': (left['center_x'] + right['center_x']) / 2,
                    'center_y': (left['center_y'] + right['center_y']) / 2,
                    'distance': distance
                })

        return pairs

    def select_target(self, red_pairs, blue_pairs, width, height):
        candidates = []

        for color, pairs in [('RED', red_pairs), ('BLUE', blue_pairs)]:
            for pair in pairs:
                target = dict(pair)
                target['color'] = color
                target['center_distance'] = np.hypot(
                    pair['center_x'] - width / 2,
                    pair['center_y'] - height / 2
                )
                candidates.append(target)

        if not candidates:
            return None

        return min(candidates, key=lambda item: item['center_distance'])

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
        except Exception as exc:
            self.get_logger().error(f'Image conversion failed: {exc}')
            return

        self.frame_count += 1
        height, width = frame.shape[:2]
        display = frame.copy()

        # 按现有算法分割红色、蓝色区域
        b, g, r = cv2.split(frame)

        red_mask = (
            (r > 150) & (r > g * 1.5) & (r > b * 1.5)
        ).astype(np.uint8) * 255

        blue_mask = (
            (b > 150) & (b > g * 1.5) & (b > r * 1.5)
        ).astype(np.uint8) * 255

        red_bars = self.find_light_bars(red_mask)
        blue_bars = self.find_light_bars(blue_mask)

        red_pairs = self.match_light_bars(red_bars)
        blue_pairs = self.match_light_bars(blue_bars)

        target = self.select_target(
            red_pairs, blue_pairs, width, height
        )

        # 图像中心参考线
        cv2.drawMarker(
            display, (width // 2, height // 2),
            (255, 255, 255), cv2.MARKER_CROSS, 24, 1
        )

        # 绘制灯条候选框
        for bars, color, label in [
            (red_bars, (0, 0, 255), 'R'),
            (blue_bars, (255, 0, 0), 'B')
        ]:
            for bar in bars:
                x, y, w, h = (
                    bar['x'], bar['y'], bar['w'], bar['h']
                )
                cv2.rectangle(
                    display, (x, y), (x + w, y + h), color, 2
                )
                cv2.putText(
                    display, label, (x, max(15, y - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1
                )

        # 绘制配对框
        for pairs, color in [
            (red_pairs, (0, 0, 255)),
            (blue_pairs, (255, 0, 0))
        ]:
            for pair in pairs:
                left, right = pair['left'], pair['right']
                x1 = min(left['x'], right['x'])
                y1 = min(left['y'], right['y'])
                x2 = max(
                    left['x'] + left['w'],
                    right['x'] + right['w']
                )
                y2 = max(
                    left['y'] + left['h'],
                    right['y'] + right['h']
                )
                cv2.rectangle(display, (x1, y1), (x2, y2), color, 2)
                cv2.circle(
                    display,
                    (int(pair['center_x']), int(pair['center_y'])),
                    4, color, -1
                )

        # 发布目标；无目标时明确标记无效
        target_msg = PointStamped()
        target_msg.header = msg.header
        if target is not None:
            target_msg.point.x = float(target['center_x'])
            target_msg.point.y = float(target['center_y'])
            target_msg.point.z = 1.0

            tx, ty = int(target['center_x']), int(target['center_y'])
            color = (0, 255, 0)
            cv2.drawMarker(
                display, (tx, ty), color,
                cv2.MARKER_TILTED_CROSS, 30, 3
            )
            cv2.putText(
                display,
                f"TARGET {target['color']} ({tx}, {ty})",
                (10, 65), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, color, 2
            )
        else:
            cv2.putText(
                display, 'TARGET: NONE', (10, 65),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2
            )

        self.target_pub.publish(target_msg)

        cv2.putText(
            display,
            f"Frame: {self.frame_count}  "
            f"Red bars/pairs: {len(red_bars)}/{len(red_pairs)}  "
            f"Blue bars/pairs: {len(blue_bars)}/{len(blue_pairs)}",
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
            0.65, (0, 255, 255), 2
        )

        try:
            cv2.imshow('SRM Armor Detection', display)
            cv2.imshow('Color Masks', cv2.bitwise_or(red_mask, blue_mask))
            cv2.waitKey(1)
        except cv2.error as exc:
            self.get_logger().error(f'OpenCV display failed: {exc}')

        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'frame={self.frame_count}, '
                f'red_bars={len(red_bars)}, red_pairs={len(red_pairs)}, '
                f'blue_bars={len(blue_bars)}, blue_pairs={len(blue_pairs)}, '
                f'target_valid={target is not None}'
            )

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArmorDetectorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
