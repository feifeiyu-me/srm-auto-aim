"""Real C++ ROS node with synthetic projections; no model, camera or control."""
from copy import deepcopy
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from srm_aim_interfaces.msg import (
    ArmorDetection, ArmorDetectionArray, ArmorLocationArray, Keypoint2D,
)


def run(calibration_file, digit_enabled):
    executable = Path(get_package_prefix('srm_armor_locator')) / (
        'lib/srm_armor_locator/armor_locator_node')
    topic = '/srm_aim/armor_detections_refined' if digit_enabled else '/srm_aim/armor_detections'
    process = subprocess.Popen([
        str(executable), '--ros-args', '-p',
        'calibration_file:=' + calibration_file, '-p',
        'enable_digit_verification:=' + str(digit_enabled).lower(), '-p',
        'detections_topic:=' + topic], start_new_session=True)
    rclpy.init()
    peer = Node('locator_test_peer')
    images = peer.create_publisher(
        Image, '/srm_aim/image_raw',
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    detections = peer.create_publisher(ArmorDetectionArray, topic, 10)
    received = {}
    subscription = peer.create_subscription(
        ArmorLocationArray, '/srm_aim/armor_locations',
        lambda m: received.update({m.header.stamp.nanosec: m}), 10)

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            assert process.poll() is None
            rclpy.spin_once(peer, timeout_sec=.01)

    def wait(predicate):
        end = time.monotonic() + 10
        while not predicate() and time.monotonic() < end:
            pump(.02)
        assert predicate(), 'ROS result timed out'

    def send(index, frame, armors):
        image = CvBridge().cv2_to_imgmsg(frame, 'mono8')
        image.header.stamp.sec = 123
        image.header.stamp.nanosec = index
        image.header.frame_id = 'source_camera'
        array = ArmorDetectionArray()
        array.header = deepcopy(image.header)
        array.image_width, array.image_height = image.width, image.height
        array.detections = armors
        # Exercise detection-first matching.
        detections.publish(array)
        pump(.05)
        images.publish(image)
        wait(lambda: index in received)
        result = received[index]
        assert result.header == image.header
        assert (result.image_width, result.image_height) == (image.width, image.height)
        assert result.optical_frame_id == 'camera_optical_frame'
        return result

    def armor(points):
        item = ArmorDetection(color=ArmorDetection.COLOR_BLUE, confidence=.875)
        for name, point in zip(('lt', 'lb', 'rb', 'rt'), points):
            setattr(item, name, Keypoint2D(x=float(point[0]), y=float(point[1])))
        return item

    try:
        wait(lambda: images.get_subscription_count() > 0
             and detections.get_subscription_count() > 0)
        pump(.5)
        c = yaml.safe_load(Path(calibration_file).read_text())
        k = np.array(c['camera_matrix']['data']).reshape(3, 3)
        d = np.array(c['distortion_coefficients']['data'])
        frame = np.zeros((c['image_height'], c['image_width']), np.uint8)
        if not digit_enabled:
            model = np.array([[-.0675, -.029, 0], [-.0675, .029, 0],
                              [.0675, .029, 0], [.0675, -.029, 0]])
            items = []
            for r, t in [([.2, .4, .1], [.3, .1, 2]), ([.5, .6, .1], [-.2, .05, 2.5])]:
                xy, _ = cv2.projectPoints(model, np.array(r), np.array(t), k, d)
                items.append(armor(xy.reshape(4, 2)))
            invalid = deepcopy(items[0])
            invalid.lt.x = -1.
            result = send(1, frame, items + [invalid])
            assert len(result.locations) == 3
            for index, depth in enumerate([2, 2.5]):
                item = result.locations[index]
                assert item.pose_valid, item.reason
                assert abs(item.axial_depth_m - depth) < 1e-4
                assert item.distance_m > item.axial_depth_m
                assert not item.digit_check_enabled and not item.digit_verified
                assert item.digit == 0 and item.digit_score == -1
                assert item.reason == 'ok_digit_unverified'
            assert not result.locations[2].pose_valid
            assert result.locations[2].reason == 'invalid_geometry'
            print('PASS multi-target PnP, invalid target, unverified bypass, source Header')
        else:
            template_path = Path(get_package_share_directory('srm_armor_locator')) / (
                'templates/digit_2_template.png')
            templ = cv2.imread(str(template_path), cv2.IMREAD_GRAYSCALE)
            assert templ is not None
            width = int(np.rint(56 * templ.shape[1] / templ.shape[0] * .6))
            scaled = cv2.resize(templ, (width, 56))
            x = 20 + (96 - width) // 2
            frame[40:96, x:x + width] = scaled
            item = armor([[20, 20], [20, 115], [115, 115], [115, 20]])
            result = send(2, frame, [item])
            assert result.locations[0].digit_check_enabled
            assert result.locations[0].digit_verified and result.locations[0].digit == 2
            assert result.locations[0].digit_score > .95
            negative = send(3, np.zeros_like(frame), [item])
            assert not negative.locations[0].digit_verified
            assert negative.locations[0].digit == 0 and not negative.locations[0].pose_valid
            assert negative.locations[0].reason == 'digit_not_verified'
            print('PASS installed template, positive/negative synthetic ROI pipeline')
        empty = send(4, np.zeros_like(frame), [])
        assert not empty.locations
        # No image for this stamp: must never use the latest frame.
        wrong = ArmorDetectionArray()
        wrong.header.stamp.sec = 123
        wrong.header.stamp.nanosec = 99
        wrong.header.frame_id = 'source_camera'
        wrong.image_width, wrong.image_height = frame.shape[1], frame.shape[0]
        detections.publish(wrong)
        pump(1.3)
        assert 99 not in received
        assert subscription
        print('PASS empty array and unmatched detection dropped')
    finally:
        peer.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
    assert process.returncode == 0


if __name__ == '__main__':
    import sys
    os.environ['ROS_DOMAIN_ID'] = '79'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    run(str(Path(sys.argv[1]).resolve()), False)
    run(str(Path(sys.argv[1]).resolve()), True)
