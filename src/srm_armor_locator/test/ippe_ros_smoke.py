"""Real C++ locator process: optional IPPE diagnostics preserve serialized results."""
from copy import deepcopy
from pathlib import Path
import math
import os
import signal
import subprocess
import time

import cv2
import numpy as np
import rclpy
import yaml
from ament_index_python.packages import get_package_prefix
from cv_bridge import CvBridge
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from srm_aim_interfaces.msg import ArmorDetection, ArmorDetectionArray, ArmorLocationArray, Keypoint2D


def run(calibration_file, enabled):
    exe = Path(get_package_prefix('srm_armor_locator')) / 'lib/srm_armor_locator/armor_locator_node'
    args = [str(exe), '--ros-args', '-p', 'calibration_file:='+calibration_file,
            '-p', 'enable_digit_verification:=false']
    if enabled:
        args += ['-p', 'ippe_debug_enabled:=true', '-p', 'ippe_debug_interval_sec:=0.2',
                 '-p', 'ippe_debug_max_samples:=2']
    process = subprocess.Popen(args, start_new_session=True)
    rclpy.init()
    peer = Node('ippe_debug_test_peer')
    images = peer.create_publisher(Image, '/srm_aim/image_raw',
                                  QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
    detections = peer.create_publisher(ArmorDetectionArray, '/srm_aim/armor_detections_refined', 10)
    outputs, reports = {}, []
    subscriptions = [
        peer.create_subscription(ArmorLocationArray, '/srm_aim/armor_locations',
                                 lambda m: outputs.update({m.header.stamp.nanosec: m}), 20),
        peer.create_subscription(DiagnosticArray, '/srm_aim/armor_locator/ippe_debug', reports.append, 20)]

    def pump(duration):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            assert process.poll() is None, 'locator exited'
            rclpy.spin_once(peer, timeout_sec=.005)

    def wait(predicate):
        end = time.monotonic()+10
        while not predicate() and time.monotonic() < end:
            pump(.02)
        assert predicate(), 'ROS result timeout'

    try:
        wait(lambda: images.get_subscription_count() and detections.get_subscription_count())
        pump(.3)
        assert bool(peer.get_publishers_info_by_topic('/srm_aim/armor_locator/ippe_debug')) == enabled
        calibration = yaml.safe_load(Path(calibration_file).read_text())
        k = np.array(calibration['camera_matrix']['data']).reshape(3, 3)
        d = np.array(calibration['distortion_coefficients']['data'])
        gray = np.zeros((calibration['image_height'], calibration['image_width']), np.uint8)
        model = np.array([[-.0675, -.029, 0], [-.0675, .029, 0], [.0675, .029, 0], [.0675, -.029, 0]])
        items = []
        for r, t in [([.1, .2, 0], [0, 0, 4.7]), ([.3, .5, .1], [.3, .1, 2])]:
            xy, _ = cv2.projectPoints(model, np.array(r), np.array(t), k, d)
            armor = ArmorDetection(color=ArmorDetection.COLOR_BLUE, confidence=.85)
            for name, point in zip(('lt', 'lb', 'rb', 'rt'), xy.reshape(4, 2)):
                setattr(armor, name, Keypoint2D(x=float(point[0]), y=float(point[1])))
            items.append(armor)
        for seq in range(1, 13):
            image = CvBridge().cv2_to_imgmsg(gray, 'mono8')
            image.header.stamp.sec = 123
            image.header.stamp.nanosec = seq
            image.header.frame_id = 'source_camera'
            array = ArmorDetectionArray(header=deepcopy(image.header), image_width=image.width,
                                        image_height=image.height, detections=items)
            detections.publish(array)
            pump(.02)
            images.publish(image)
            wait(lambda: seq in outputs)
        pump(.4)
        assert len(outputs) == 12
        assert any(m.locations[0].ambiguous for m in outputs.values())
        for m in outputs.values():
            assert m.locations[0].reason == 'ambiguous_ippe'
            assert m.locations[1].pose_valid
            assert all(not v.digit_verified and v.digit == 0 for v in m.locations)
        if enabled:
            assert reports
            samples = 0
            for report in reports:
                assert len(report.status) <= 3
                assert report.header.frame_id == 'source_camera'
                for sample in report.status[1:]:
                    values = {v.key: v.value for v in sample.values}
                    samples += 1
                    assert values['frame_id'] == 'source_camera'
                    assert values['stamp_sec'] == '123'
                    assert 1 <= int(values['stamp_nanosec']) <= 12
                    for candidate in (0, 1):
                        assert math.isfinite(float(values[f'candidate_{candidate}_reprojection_error_px']))
                        assert 1000 < float(values[f'candidate_{candidate}_t_z_mm']) < 6000
                    if values['ambiguous'] == 'true':
                        assert values['error_gap_close'] == 'true'
                        assert values['trigger'] in ('rotation', 'translation', 'rotation_and_translation')
            last = {v.key: v.value for v in reports[-1].status[0].values}
            assert last['total_attempts'] == '24'
            assert last['total_accepted'] == '12' and last['total_ambiguous_ippe'] == '12'
            assert int(last['total_samples_omitted']) > 0 and samples > 0
            print('PASS bounded/rate-limited IPPE details, original Header, mm/degree fields and cumulative 24/12/12 stats')
        else:
            assert not reports
            print('PASS default-off: no IPPE diagnostic publisher or reports')
        assert subscriptions
        return outputs
    finally:
        peer.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
        assert process.returncode == 0


if __name__ == '__main__':
    import sys
    os.environ.setdefault('ROS_DOMAIN_ID', '82')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    calibration = str(Path(sys.argv[1]).resolve())
    off = run(calibration, False)
    on = run(calibration, True)
    assert off == on, 'diagnostics changed localization messages'
    print('PASS all 12 localization arrays byte-field-equivalent with diagnostics off/on')
