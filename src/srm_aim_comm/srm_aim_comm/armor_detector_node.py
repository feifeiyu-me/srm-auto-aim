import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ArmorDetectorNode(Node):
    def __init__(self):
        super().__init__('armor_detector_node')
        self.bridge = CvBridge()
        self.frame_count = 0
        from pathlib import Path
        template_path = Path.home() / (
            'srm_aim_ws/src/srm_aim_comm/srm_aim_comm/'
            'digit_2_template.png'
        )
        self.digit_template = cv2.imread(
            str(template_path), cv2.IMREAD_GRAYSCALE
        )
        if self.digit_template is None:
            raise RuntimeError(f'Cannot load template: {template_path}')
        self.digit_threshold = float(
            self.declare_parameter('digit_threshold', 0.65).value
        )

        self.subscription = self.create_subscription(
            Image, '/srm_aim/image_raw',
            self.image_callback, qos_profile_sensor_data
        )
        self.target_pub = self.create_publisher(
            PointStamped, '/srm_aim/target', 10
        )

        cv2.namedWindow('SRM Armor Detection', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Color Masks', cv2.WINDOW_NORMAL)
        cv2.createTrackbar('Brightness', 'Color Masks', 113, 255, lambda _: None)
        cv2.createTrackbar('Color diff', 'Color Masks', 60, 150, lambda _: None)

        self.get_logger().info(
            'Detector started. Masks: RED on left, BLUE on right.'
        )

    def make_masks(self, frame):
        brightness = cv2.getTrackbarPos('Brightness', 'Color Masks')
        difference = cv2.getTrackbarPos('Color diff', 'Color Masks')

        # 转有符号数，避免颜色相减发生 uint8 溢出。
        b, g, r = [
            channel.astype(np.int16)
            for channel in cv2.split(frame)
        ]

        red = (
            (r > brightness)
            & (r - b > difference)
            & (r - g > difference // 2)
        ).astype(np.uint8) * 255

        blue = (
            (b > brightness)
            & (b - r > difference)
            & (b - g > difference // 2)
        ).astype(np.uint8) * 255

        # 只沿竖直方向连接小断点，减少横向粘连。
        kernel = np.ones((3, 1), np.uint8)
        red = cv2.morphologyEx(red, cv2.MORPH_CLOSE, kernel)
        blue = cv2.morphologyEx(blue, cv2.MORPH_CLOSE, kernel)
        return red, blue

    def find_light_bars(self, mask):
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        bars = []

        for contour in contours:
            # 细线轮廓的面积可能接近零，不能只按面积剔除。
            if len(contour) < 2:
                continue

            rect = cv2.minAreaRect(contour)
            box = cv2.boxPoints(rect)
            edge1 = box[1] - box[0]
            edge2 = box[2] - box[1]
            size1 = float(np.linalg.norm(edge1))
            size2 = float(np.linalg.norm(edge2))
            length = max(size1, size2)
            width = min(size1, size2)

            if length < 6:
                continue
            if length / max(width, 1.0) < 1.8:
                continue

            axis = edge1 if size1 >= size2 else edge2
            # 统一让长轴指向图像下方。
            if axis[1] < 0:
                axis = -axis

            tilt = math.degrees(math.atan2(float(axis[0]), float(axis[1])))
            if abs(tilt) > 40:
                continue

            bars.append({
                'center': np.array(rect[0], dtype=np.float32),
                'length': length,
                'width': width,
                'tilt': tilt,
                'box': box
            })

        return bars, len(contours)

    def match_light_bars(self, bars):
        pairs = []

        for i, first in enumerate(bars):
            for second in bars[i + 1:]:
                left, right = sorted(
                    [first, second], key=lambda bar: bar['center'][0]
                )

                length = (left['length'] + right['length']) / 2
                length_ratio = (
                    min(left['length'], right['length'])
                    / max(left['length'], right['length'])
                )
                if length_ratio < 0.55:
                    continue

                angle_diff = abs(left['tilt'] - right['tilt'])
                if angle_diff > 20:
                    continue

                delta = right['center'] - left['center']
                # 沿灯条长轴、垂直长轴分别检查偏移和间距。
                tilt = math.radians((left['tilt'] + right['tilt']) / 2)
                axis = np.array([math.sin(tilt), math.cos(tilt)])
                normal = np.array([math.cos(tilt), -math.sin(tilt)])

                along = abs(float(np.dot(delta, axis)))
                across = abs(float(np.dot(delta, normal)))
                spacing = across / length

                if along / length > 0.5:
                    continue
                if not 0.6 <= spacing <= 5.0:
                    continue

                pairs.append({
                    'left': left,
                    'right': right,
                    'center': (left['center'] + right['center']) / 2,
                    'quality': (
                        1 - length_ratio
                        + angle_diff / 20
                        + along / length
                    )
                })

        return pairs


    def digit_score(self, frame, pair):
        left = pair['left']
        right = pair['right']
        length = (left['length'] + right['length']) / 2
        tilt = math.radians((left['tilt'] + right['tilt']) / 2)
        axis = np.array([math.sin(tilt), math.cos(tilt)],
                        dtype=np.float32)

        # 数字比灯条更高，向上下扩展；沿两灯条校正倾斜。
        top_left = left['center'] - axis * length
        top_right = right['center'] - axis * length
        bottom_left = left['center'] + axis * length
        source = np.float32([top_left, top_right, bottom_left])
        destination = np.float32([[0, 0], [95, 0], [0, 95]])
        matrix = cv2.getAffineTransform(source, destination)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        roi = cv2.warpAffine(gray, matrix, (96, 96))

        # 排除灯条，仅在板内区域搜索数字。
        roi = roi[:, 14:82]
        best = 0.0
        template = self.digit_template
        aspect = template.shape[1] / template.shape[0]

        # 同时适应数字大小变化和侧视时的水平压缩。
        for height in (44, 56, 68, 80, 90):
            for squeeze in (0.6, 0.8, 1.0):
                width = round(height * aspect * squeeze)
                if width < 8 or width > roi.shape[1]:
                    continue
                candidate = cv2.resize(template, (width, height))
                if candidate.std() < 2:
                    continue

                scores = cv2.matchTemplate(
                    roi, candidate, cv2.TM_CCOEFF_NORMED
                )
                # 只接受接近两灯条中间的数字，避免跨板大框
                # 在边缘搜索到旁边真实装甲板上的数字。
                for y in range(scores.shape[0]):
                    for x in range(scores.shape[1]):
                        if abs(x + width / 2 - roi.shape[1] / 2) > 9:
                            continue
                        if abs(y + height / 2 - 48) > 18:
                            continue
                        value = float(scores[y, x])
                        if math.isfinite(value):
                            best = max(best, value)
        return best

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
        red_mask, blue_mask = self.make_masks(frame)

        candidates = []
        statistics = []

        for label, mask, color in [
            ('RED', red_mask, (0, 0, 255)),
            ('BLUE', blue_mask, (255, 0, 0))
        ]:
            bars, contour_count = self.find_light_bars(mask)
            pairs = self.match_light_bars(bars)
            statistics.append((label, contour_count, len(bars), len(pairs)))

            for bar in bars:
                box = np.round(bar['box']).astype(np.int32)
                cv2.polylines(display, [box], True, color, 2)

            for pair in pairs:
                score = self.digit_score(frame, pair)
                px, py = pair['center']
                accepted = score >= self.digit_threshold
                cv2.putText(
                    display, f'2:{score:.2f}',
                    (int(px), max(15, int(py) - 18)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 255, 0) if accepted else (0, 165, 255), 1
                )
                if not accepted:
                    continue
                pair['color'] = label
                candidates.append(pair)
                left = tuple(np.round(pair['left']['center']).astype(int))
                right = tuple(np.round(pair['right']['center']).astype(int))
                cv2.line(display, left, right, color, 2)

        image_center = np.array([width / 2, height / 2])

        # 当前仍按画面中心优先选择红/蓝候选，尚未接敌方颜色配置。
        target = min(
            candidates,
            key=lambda item: (
                float(np.linalg.norm(item['center'] - image_center))
                / math.hypot(width, height)
                + 0.05 * item['quality']
            ),
            default=None
        )

        target_msg = PointStamped()
        target_msg.header = msg.header

        if target is not None:
            x, y = target['center']
            target_msg.point.x = float(x)
            target_msg.point.y = float(y)
            target_msg.point.z = 1.0

            cv2.drawMarker(
                display, (int(x), int(y)), (0, 255, 0),
                cv2.MARKER_TILTED_CROSS, 26, 2
            )
            target_text = f"TARGET {target['color']} ({x:.1f}, {y:.1f})"
        else:
            # 默认 z=0，明确发布本帧无有效目标，不沿用旧目标。
            target_text = 'TARGET: NONE'

        self.target_pub.publish(target_msg)

        cv2.drawMarker(
            display, (width // 2, height // 2),
            (255, 255, 255), cv2.MARKER_CROSS, 24, 1
        )

        for index, (label, contours, bars, pairs) in enumerate(statistics):
            text = f'{label}: contours={contours} bars={bars} pairs={pairs}'
            cv2.putText(
                display, text, (10, 30 + index * 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2
            )

        cv2.putText(
            display, target_text, (10, 95),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )

        cv2.imshow('SRM Armor Detection', display)
        # 左侧红色掩膜，右侧蓝色掩膜。
        cv2.imshow('Color Masks', np.hstack([red_mask, blue_mask]))
        cv2.waitKey(1)

        if self.frame_count % 60 == 0:
            self.get_logger().info(
                f'{statistics}; target_valid={target is not None}'
            )

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArmorDetectorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
