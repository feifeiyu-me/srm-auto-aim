import math
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from std_msgs.msg import Bool, Float32
from srm_aim_comm.armor_pnp import ArmorPnP
from srm_aim_comm.armor_corner_refiner import refine_pair
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


class ArmorDetectorNode(Node):
    def __init__(self):
        super().__init__('armor_detector_node')
        self.bridge = CvBridge()
        self.frame_count = 0
        self.previous_target = None

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
        self.hold_margin = float(
            self.declare_parameter('hold_margin', 0.05).value
        )
        self.association_timeout = 0.15

        if not (
            math.isfinite(self.digit_threshold)
            and 0.0 <= self.digit_threshold <= 1.0
            and math.isfinite(self.hold_margin)
            and 0.0 <= self.hold_margin <= self.digit_threshold
        ):
            raise ValueError('Invalid digit_threshold or hold_margin')

        self.subscription = self.create_subscription(
            Image, '/srm_aim/image_raw',
            self.image_callback, qos_profile_sensor_data
        )
        self.target_pub = self.create_publisher(
            PointStamped, '/srm_aim/target', 10
        )

        calibration = self.declare_parameter(
            'calibration_file',
            str(Path.home() / 'srm_aim_ws/calibration/20261019_182722/camera.yaml')
        ).value
        self.pnp = ArmorPnP(calibration, width_m=0.135, height_m=0.058)
        self.enable_pca_refine = bool(
            self.declare_parameter('enable_pca_refine', True).value
        )
        self.pnp_frame = self.declare_parameter(
            'pnp_frame_id', 'camera_optical_frame'
        ).value
        self.pose_pub = self.create_publisher(
            PoseStamped, '/srm_aim/armor_pose', 10)
        self.pnp_valid_pub = self.create_publisher(
            Bool, '/srm_aim/pnp_valid', 10)
        self.pnp_error_pub = self.create_publisher(
            Float32, '/srm_aim/pnp_error_px', 10)

        cv2.namedWindow('SRM Armor Detection', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Color Masks', cv2.WINDOW_NORMAL)
        cv2.createTrackbar(
            'Brightness', 'Color Masks', 113, 255, lambda _: None
        )
        cv2.createTrackbar(
            'Color diff', 'Color Masks', 60, 150, lambda _: None
        )
        self.get_logger().info(
            f'Detector started: acquire={self.digit_threshold:.2f}, '
            f'hold={self.digit_threshold - self.hold_margin:.2f}. '
            'Masks: RED left, BLUE right.'
        )

    def make_masks(self, frame):
        brightness = cv2.getTrackbarPos('Brightness', 'Color Masks')
        difference = cv2.getTrackbarPos('Color diff', 'Color Masks')
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

            if length < 6 or length / max(width, 1.0) < 1.8:
                continue

            axis = edge1 if size1 >= size2 else edge2
            if axis[1] < 0:
                axis = -axis
            tilt = math.degrees(
                math.atan2(float(axis[0]), float(axis[1]))
            )
            if abs(tilt) > 40:
                continue

            bars.append({
                'center': np.array(rect[0], dtype=np.float32),
                'length': length,
                'width': width,
                'tilt': tilt,
                'box': box,
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
                tilt = math.radians(
                    (left['tilt'] + right['tilt']) / 2
                )
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
                    ),
                })
        return pairs

    def digit_score(self, gray, pair):
        left, right = pair['left'], pair['right']
        length = (left['length'] + right['length']) / 2
        tilt = math.radians((left['tilt'] + right['tilt']) / 2)
        axis = np.array(
            [math.sin(tilt), math.cos(tilt)], dtype=np.float32
        )

        top_left = left['center'] - axis * length
        top_right = right['center'] - axis * length
        bottom_left = left['center'] + axis * length
        source = np.float32([top_left, top_right, bottom_left])
        destination = np.float32([[0, 0], [95, 0], [0, 95]])
        matrix = cv2.getAffineTransform(source, destination)
        roi = cv2.warpAffine(gray, matrix, (96, 96))[:, 14:82]

        best = 0.0
        template = self.digit_template
        aspect = template.shape[1] / template.shape[0]

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
                # 保留原来的中心位置限制，用数组一次筛选。
                ys, xs = np.indices(scores.shape)
                valid = (
                    (np.abs(xs + width / 2 - roi.shape[1] / 2) <= 9)
                    & (np.abs(ys + height / 2 - 48) <= 18)
                    & np.isfinite(scores)
                )
                if np.any(valid):
                    best = max(best, float(np.max(scores[valid])))
        return best

    def association_distance(self, pair, stamp_ns):
        previous = self.previous_target
        if previous is None:
            return None

        dt = (stamp_ns - previous['stamp_ns']) * 1e-9
        if not 0.0 < dt <= self.association_timeout:
            return None
        if pair['color'] != previous['color']:
            return None

        length = (
            pair['left']['length'] + pair['right']['length']
        ) / 2
        old_length = previous['length']
        if not 0.7 <= length / old_length <= 1.4:
            return None

        distance = float(
            np.linalg.norm(pair['center'] - previous['center'])
        )
        if distance > 0.5 * old_length:
            return None
        return distance / old_length

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
        except Exception as exc:
            self.previous_target = None
            self.get_logger().error(f'Image conversion failed: {exc}')
            return

        self.frame_count += 1
        height, width = frame.shape[:2]
        display = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        red_mask, blue_mask = self.make_masks(frame)
        stamp_ns = (
            msg.header.stamp.sec * 1000000000
            + msg.header.stamp.nanosec
        )

        candidates = []
        statistics = []
        for label, mask, color in [
            ('RED', red_mask, (0, 0, 255)),
            ('BLUE', blue_mask, (255, 0, 0)),
        ]:
            bars, contour_count = self.find_light_bars(mask)
            pairs = self.match_light_bars(bars)
            statistics.append(
                (label, contour_count, len(bars), len(pairs))
            )

            for bar in bars:
                box = np.round(bar['box']).astype(np.int32)
                cv2.polylines(display, [box], True, color, 2)

            for pair in pairs:
                pair['color'] = label
                pair['score'] = self.digit_score(gray, pair)
                association = self.association_distance(pair, stamp_ns)
                pair['association'] = association
                threshold = self.digit_threshold
                if association is not None:
                    threshold -= self.hold_margin

                accepted = pair['score'] >= threshold
                px, py = pair['center']
                cv2.putText(
                    display, f"2:{pair['score']:.2f}",
                    (int(px), max(15, int(py) - 18)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    (0, 255, 0) if accepted else (0, 165, 255), 1
                )
                if accepted:
                    candidates.append(pair)

        image_center = np.array([width / 2, height / 2])
        associated = [
            item for item in candidates
            if item['association'] is not None
        ]
        if associated:
            # 优先维持上一帧目标，不因靠近画面中心而随意换板。
            target = min(
                associated,
                key=lambda item: (
                    item['association'] + 0.05 * item['quality']
                )
            )
        else:
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
        self.previous_target = None

        if target is not None:
            x, y = target['center']
            target_msg.point.x = float(x)
            target_msg.point.y = float(y)
            target_msg.point.z = 1.0

            length = (
                target['left']['length']
                + target['right']['length']
            ) / 2
            self.previous_target = {
                'center': target['center'].copy(),
                'color': target['color'],
                'length': length,
                'stamp_ns': stamp_ns,
            }

            # 绿色框用于显示选择结果，不作为 PnP 四角点。
            points = np.vstack([
                target['left']['box'], target['right']['box']
            ])
            hull = cv2.convexHull(
                np.round(points).astype(np.int32)
            )
            cv2.polylines(display, [hull], True, (0, 255, 0), 2)
            cv2.drawMarker(
                display, (int(x), int(y)), (0, 255, 0),
                cv2.MARKER_TILTED_CROSS, 26, 2
            )

            state = (
                'FOLLOW'
                if target['association'] is not None else 'ACQUIRE'
            )
            target_text = (
                f"TARGET {target['color']} {state} "
                f"score={target['score']:.2f}"
            )
        else:
            # 本帧未通过筛选：发布无效，不沿用旧位置。
            target_text = 'TARGET: NONE'

        self.publish_pnp(target, msg.header, display, width, height, gray)
        self.target_pub.publish(target_msg)
        cv2.drawMarker(
            display, (width // 2, height // 2),
            (255, 255, 255), cv2.MARKER_CROSS, 24, 1
        )
        for index, (label, contours, bars, pairs) in enumerate(statistics):
            cv2.putText(
                display,
                f'{label}: contours={contours} bars={bars} pairs={pairs}',
                (10, 30 + index * 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2
            )
        cv2.putText(
            display, target_text, (10, 95),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2
        )
        cv2.imshow('SRM Armor Detection', display)
        cv2.imshow('Color Masks', np.hstack([red_mask, blue_mask]))
        cv2.waitKey(1)

        if self.frame_count % 60 == 0:
            self.get_logger().info(
                f'{statistics}; {target_text}'
            )

    def publish_pnp(self, target, header, display, width, height, gray):
        valid = Bool()
        text = 'PnP: NO TARGET'
        color = (0, 165, 255)

        if (width, height) != self.pnp.size:
            text = 'PnP: RESOLUTION MISMATCH'
        elif target is not None:
            corners = self.pnp.corners_from_pair(target)
            mode = 'RAW'
            if self.enable_pca_refine:
                refined = refine_pair(gray, target)
                if refined is None:
                    mode = 'PCA_REJECT'
                else:
                    corners = refined
                    mode = 'PCA'
            estimate = None
            if mode != 'PCA_REJECT':
                estimate = self.pnp.solve(corners, (width, height))

            for point in corners:
                cv2.circle(
                    display, tuple(np.round(point).astype(int)),
                    3, (0, 255, 255), -1
                )

            text = f'PnP {mode}: INVALID CORNERS / POSE'
            if estimate is not None:
                pose = PoseStamped()
                pose.header.stamp = header.stamp
                pose.header.frame_id = self.pnp_frame

                position = pose.pose.position
                position.x, position.y, position.z = map(
                    float, estimate['position']
                )
                q = pose.pose.orientation
                q.x, q.y, q.z, q.w = map(
                    float, estimate['quaternion']
                )
                self.pose_pub.publish(pose)

                error = Float32()
                error.data = float(estimate['error_px'])
                self.pnp_error_pub.publish(error)
                valid.data = True
                color = (0, 255, 0)
                text = (
                    f"PnP {mode}: {estimate['distance_m']:.3f}m "
                    f"error={estimate['error_px']:.2f}px"
                )
                xyz = (
                    f'X={position.x:.3f} Y={position.y:.3f} '
                    f'Z={position.z:.3f} m'
                )
                cv2.putText(
                    display, xyz, (10, 155),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2
                )

        self.pnp_valid_pub.publish(valid)
        cv2.putText(
            display, text, (10, 125),
            cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
