"""Run the real C++ executable with synthetic ROS images; no hardware or YOLO."""

from copy import deepcopy
import os
from pathlib import Path
import signal
import subprocess
import time

import cv2
import numpy as np
import rclpy
from ament_index_python.packages import get_package_prefix
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from srm_aim_interfaces.msg import ArmorDetection, ArmorDetectionArray, Keypoint2D


def main():
    # Keep this synthetic-data test away from live robot topic graphs.
    os.environ['ROS_DOMAIN_ID'] = '77'
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    executable = Path(get_package_prefix('srm_armor_refiner')) / (
        'lib/srm_armor_refiner/armor_refiner_node')
    process = subprocess.Popen([str(executable)], start_new_session=True)
    rclpy.init()
    peer = Node('refiner_smoke_peer')
    images = peer.create_publisher(
        Image, '/srm_aim/image_raw',
        QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    detections = peer.create_publisher(ArmorDetectionArray, '/srm_aim/armor_detections', 10)
    outputs, debug = {}, {}
    subscriptions = [
        peer.create_subscription(ArmorDetectionArray, '/srm_aim/armor_detections_refined',
                                 lambda m: outputs.update({m.header.stamp.nanosec: m}), 10),
        peer.create_subscription(
            DiagnosticArray, '/srm_aim/armor_refiner/debug',
            lambda m: debug.setdefault(m.header.stamp.nanosec, []).append(m), 100),
    ]

    def pump(seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            assert process.poll() is None, 'C++ node exited early'
            rclpy.spin_once(peer, timeout_sec=0.01)

    def wait_for(predicate, seconds=5):
        end = time.monotonic() + seconds
        while not predicate() and time.monotonic() < end:
            pump(0.02)
        assert predicate(), 'Timed out waiting for ROS data'

    def image(index, bright=True):
        gray = np.zeros((240, 480), np.uint8)
        if bright:
            cv2.line(gray, (100, 80), (100, 160), 220, 5)
            cv2.line(gray, (200, 80), (200, 160), 220, 5)
        message = CvBridge().cv2_to_imgmsg(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), 'bgr8')
        message.header.stamp.sec = 123
        message.header.stamp.nanosec = index
        message.header.frame_id = 'camera'
        return message

    def result(source):
        message = ArmorDetectionArray()
        message.header = deepcopy(source.header)
        message.image_width, message.image_height = source.width, source.height
        for x, color in [(102., ArmorDetection.COLOR_BLUE), (340., ArmorDetection.COLOR_RED)]:
            message.detections.append(ArmorDetection(
                color=color, confidence=0.875,
                lt=Keypoint2D(x=x, y=81.), lb=Keypoint2D(x=x, y=161.),
                rb=Keypoint2D(x=x + 100, y=161.), rt=Keypoint2D(x=x + 100, y=81.)))
        invalid = deepcopy(message.detections[0])
        invalid.lt.x = -1.0
        message.detections.append(invalid)
        return message

    def key_values(status):
        return {item.key: item.value for item in status.values}

    try:
        wait_for(lambda: images.get_subscription_count() > 0
                 and detections.get_subscription_count() > 0)
        pump(0.3)
        old, latest = image(1), image(2, False)
        images.publish(old)
        pump(0.05)
        images.publish(latest)
        pump(0.05)
        source = result(old)
        detections.publish(source)
        wait_for(lambda: 1 in outputs and 1 in debug)
        output = outputs[1]
        assert output.header == old.header
        assert (output.image_width, output.image_height) == (480, 240)
        assert len(output.detections) == 2
        assert output.detections[0].color == ArmorDetection.COLOR_BLUE
        assert output.detections[0].confidence == source.detections[0].confidence
        assert abs(output.detections[0].lt.x - 100) < 0.1
        assert output.detections[0].lt.y < output.detections[0].lb.y
        assert output.detections[0].rt.y < output.detections[0].rb.y
        assert output.detections[1] == source.detections[1]
        statuses = debug[1][0].status
        assert [key_values(s)['outcome'] for s in statuses[:-1]] == [
            'refined', 'fallback', 'dropped']
        assert statuses[1].message == 'left_low_contrast'
        assert statuses[2].message == 'invalid_yolo_geometry'
        print('PASS exact old frame; multi-target refine/fallback/drop; preserved metadata')

        empty = result(latest)
        empty.detections = []
        detections.publish(empty)
        wait_for(lambda: 2 in outputs)
        assert outputs[2].header == latest.header and not outputs[2].detections
        print('PASS matched empty array')

        source = image(3)
        detections.publish(result(source))
        pump(0.05)
        images.publish(source)
        wait_for(lambda: 3 in outputs)
        assert abs(outputs[3].detections[0].lt.x - 100) < 0.1
        print('PASS detection-before-image')

        for index, mismatch in [(4, 'stamp'), (6, 'frame_id'), (8, 'size')]:
            source = image(index)
            wrong = result(source)
            if mismatch == 'stamp':
                wrong.header.stamp.nanosec += 1
            elif mismatch == 'frame_id':
                wrong.header.frame_id = 'other'
            else:
                wrong.image_width += 1
            images.publish(source)
            pump(0.05)
            detections.publish(wrong)
            pump(1.2)
            assert index not in outputs and wrong.header.stamp.nanosec not in outputs
        print('PASS stamp/frame_id/size mismatch: no refined frame emitted')
        assert any(s.message == 'unmatched_detection_expired'
                   for values in debug.values() for message in values for s in message.status)
        assert any(s.message == 'image_size_mismatch'
                   for values in debug.values() for message in values for s in message.status)
        published = dict(peer.get_publisher_names_and_types_by_node('armor_refiner_node', '/'))
        assert '/srm_aim/armor_detections' not in published
        assert '/srm_aim/armor_detections_refined' in published
        assert subscriptions
        print('PASS original YOLO topic untouched; diagnostic reasons available')
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
    print('PASS clean C++ node exit; no hardware nodes started')


if __name__ == '__main__':
    main()
