"""Offline adapter and ROS callback tests; no real model or camera required."""

import sys
import time
from types import SimpleNamespace
from unittest.mock import Mock

import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from srm_aim_interfaces.msg import ArmorDetection, ArmorDetectionArray
from srm_aim_comm.yolo_pose_adapter import result_to_message, validate_model
from srm_aim_comm.yolo_pose_node import YoloPoseNode


POINTS = np.array([[20., 20.], [20., 70.], [80., 70.], [80., 20.]])


def result(count=1):
    return SimpleNamespace(
        orig_shape=(100, 160),
        boxes=SimpleNamespace(conf=np.full(count, 0.9), cls=np.arange(count) % 2),
        keypoints=SimpleNamespace(
            xy=np.repeat(POINTS[None], count, axis=0),
            conf=np.full((count, 4), 0.8)))


def header():
    value = Header(frame_id='camera')
    value.stamp.sec, value.stamp.nanosec = 123, 456789012
    return value


def convert(value, **kwargs):
    return result_to_message(value, header(), 160, 100, **kwargs)


def test_multiple_targets_preserve_header_fields_classes_and_corner_order():
    source_header = header()
    message = result_to_message(result(2), source_header, 160, 100)
    assert message.header == source_header
    source_header.stamp.sec = 999
    assert message.header.stamp.sec == 123
    assert (message.image_width, message.image_height) == (160, 100)
    assert [d.color for d in message.detections] == [
        ArmorDetection.COLOR_BLUE, ArmorDetection.COLOR_RED]
    for detection in message.detections:
        assert detection.confidence == pytest.approx(0.9)
        assert [[getattr(detection, field).x, getattr(detection, field).y]
                for field in ('lt', 'lb', 'rb', 'rt')] == POINTS.tolist()


def test_empty_result_and_all_filtered_result_publish_stamped_empty_array():
    empty = result(0)
    empty.keypoints = None
    for value in (empty, result(2)):
        message = convert(value, confidence=0.95)
        assert message.header == header()
        assert message.image_width == 160 and message.image_height == 100
        assert not message.detections


def test_explicit_model_annotation_reordering():
    value = result()
    value.keypoints.xy = value.keypoints.xy[:, [3, 0, 1, 2]]
    message = convert(value, corner_indices=[1, 2, 3, 0])
    assert message.detections[0].lt.x == 20
    assert message.detections[0].rt.y == 20


@pytest.mark.parametrize('points', [
    [[np.nan, 20], [20, 70], [80, 70], [80, 20]],
    [[np.inf, 20], [20, 70], [80, 70], [80, 20]],
    [[-1, 20], [20, 70], [80, 70], [80, 20]],
    [[160, 20], [20, 70], [80, 70], [80, 20]],
    [[20, 100], [20, 70], [80, 70], [80, 20]],
    [[20, 20], [20, 20], [80, 70], [80, 20]],
    [[20, 20], [80, 70], [20, 70], [80, 20]],
    [[20, 20], [20, 70], [30, 30], [80, 20]],
    [[20, 20], [80, 20], [80, 70], [20, 70]],
    [[20, 20], [20, 21], [21, 21], [21, 20]],
    [[20, 20], [20, 23], [150, 23], [150, 20]],
])
def test_rejects_invalid_degenerate_concave_crossed_and_implausible_quads(points):
    value = result(2)
    value.keypoints.xy[0] = points
    message = convert(value)
    assert len(message.detections) == 1
    assert message.detections[0].color == ArmorDetection.COLOR_RED


@pytest.mark.parametrize('score', [0.49, np.nan, np.inf, 1.01])
def test_rejects_low_missing_or_nonfinite_keypoint_confidence(score):
    value = result()
    value.keypoints.conf[0, 2] = score
    assert not convert(value).detections


@pytest.mark.parametrize('score,label', [
    (np.nan, 0), (1.01, 0), (0.2, 0), (0.9, 2), (0.9, 0.5), (0.9, np.nan)])
def test_rejects_invalid_detection_confidence_and_unknown_classes(score, label):
    value = result()
    value.boxes.conf[0] = score
    value.boxes.cls = np.array([label], dtype=float)
    assert not convert(value).detections


def test_border_origin_is_not_a_missing_point_sentinel():
    value = result()
    value.keypoints.xy -= [20, 20]
    assert len(convert(value).detections) == 1


@pytest.mark.parametrize('failure', ['missing', 'confidence', 'shape', 'image_shape'])
def test_malformed_model_output_is_an_error_not_a_normal_empty_result(failure):
    value = result()
    if failure == 'missing':
        value.keypoints = None
    elif failure == 'confidence':
        value.keypoints.conf = None
    elif failure == 'shape':
        value.keypoints.xy = value.keypoints.xy[:, :3]
    else:
        value.orig_shape = (10, 16)
    with pytest.raises(ValueError):
        convert(value)


def model():
    return SimpleNamespace(task='pose', model=SimpleNamespace(kpt_shape=[4, 3]),
                           names={0: 'blue', 1: 'red'},
                           predict=Mock(return_value=[result()]))


def test_rejects_person_pose_and_wrong_class_mapping():
    value = model()
    validate_model(value)
    value.model.kpt_shape = [17, 3]
    with pytest.raises(ValueError, match='kpt_shape'):
        validate_model(value)
    value.model.kpt_shape = [4, 3]
    value.names = {0: 'red', 1: 'blue'}
    with pytest.raises(ValueError, match='classes'):
        validate_model(value)


@pytest.fixture
def ros_model(tmp_path, monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '43')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    weights = tmp_path / 'best.pt'
    weights.write_bytes(b'mocked model; not a real checkpoint')
    fake = model()
    loader = Mock(return_value=fake)
    monkeypatch.setitem(sys.modules, 'ultralytics', SimpleNamespace(YOLO=loader))
    rclpy.init(args=['--ros-args', '-p', f'model_path:={weights}'])
    yield fake, loader, weights
    rclpy.shutdown()


def image():
    message = CvBridge().cv2_to_imgmsg(np.zeros((100, 160, 3), np.uint8), 'bgr8')
    message.header = header()
    return message


def test_callback_empty_and_multiple_detections(ros_model):
    fake, _, _ = ros_model
    node = YoloPoseNode()
    try:
        node.publisher = Mock()
        for count in (0, 2):
            fake.predict.return_value = [result(count)]
            source = image()
            node.image_callback(source)
            message = node.publisher.publish.call_args[0][0]
            assert message.header == source.header
            assert len(message.detections) == count
            assert message.image_width == source.width
        assert node.publisher.publish.call_count == 2
    finally:
        node.destroy_node()


@pytest.mark.parametrize('encoding,channels', [
    ('bgr8', 3), ('rgb8', 3), ('mono8', 1), ('bgra8', 4), ('rgba8', 4)])
def test_supported_encodings_are_converted_to_bgr(ros_model, encoding, channels):
    fake, _, _ = ros_model
    node = YoloPoseNode()
    try:
        node.publisher = Mock()
        shape = (100, 160) if channels == 1 else (100, 160, channels)
        source = CvBridge().cv2_to_imgmsg(np.zeros(shape, np.uint8), encoding)
        source.header = header()
        node.image_callback(source)
        assert fake.predict.call_args.kwargs['source'].shape == (100, 160, 3)
        assert node.publisher.publish.call_args[0][0].header == source.header
    finally:
        node.destroy_node()


@pytest.mark.parametrize('failure', ['empty', 'encoding', 'inference', 'bad_result'])
def test_callback_errors_do_not_publish_false_negative_results(ros_model, failure):
    fake, _, _ = ros_model
    node = YoloPoseNode()
    try:
        node.publisher = Mock()
        logger = Mock()
        node.get_logger = lambda: logger
        source = image()
        if failure == 'empty':
            source.data = []
        elif failure == 'encoding':
            source.encoding = '32FC1'
        elif failure == 'inference':
            fake.predict.side_effect = RuntimeError('mock inference failure')
        else:
            fake.predict.return_value = []
        node.image_callback(source)
        node.publisher.publish.assert_not_called()
        logger.error.assert_called_once()
    finally:
        node.destroy_node()


def test_slow_inference_warns_without_replacing_timestamp(ros_model):
    fake, _, _ = ros_model
    node = YoloPoseNode()
    try:
        node.publisher = Mock()
        logger = Mock()
        node.get_logger = lambda: logger

        def slow_predict(**kwargs):
            time.sleep(0.01)
            return [result()]
        fake.predict.side_effect = slow_predict
        node.warn_inference_sec = 0.001
        node.image_callback(image())
        logger.warning.assert_called_once()
        assert node.publisher.publish.call_args[0][0].header == header()
    finally:
        node.destroy_node()


def test_missing_model_stops_before_loader_and_does_not_fallback(ros_model):
    _, loader, weights = ros_model
    weights.unlink()
    with pytest.raises(RuntimeError, match='existing local .pt'):
        YoloPoseNode()
    loader.assert_not_called()


def test_loader_failure_prevents_startup(ros_model):
    _, loader, _ = ros_model
    loader.side_effect = RuntimeError('invalid checkpoint')
    with pytest.raises(RuntimeError, match='invalid checkpoint'):
        YoloPoseNode()


def test_incompatible_model_prevents_startup(ros_model):
    fake, _, _ = ros_model
    fake.model.kpt_shape = [17, 3]
    with pytest.raises(RuntimeError, match='kpt_shape'):
        YoloPoseNode()


def test_real_ros_image_subscription_and_detection_publication_with_mock_model(ros_model):
    node = YoloPoseNode()
    peer = Node('offline_yolo_test_peer', use_global_arguments=False)
    publisher = peer.create_publisher(Image, '/srm_aim/image_raw', qos_profile_sensor_data)
    received = []
    subscription = peer.create_subscription(
        ArmorDetectionArray, '/srm_aim/armor_detections', received.append, 1)
    source = image()
    try:
        deadline = time.monotonic() + 5.0
        while not received and time.monotonic() < deadline:
            publisher.publish(source)
            rclpy.spin_once(node, timeout_sec=0.05)
            rclpy.spin_once(peer, timeout_sec=0.05)
        assert received
        assert received[-1].header == source.header
        assert received[-1].detections[0].color == ArmorDetection.COLOR_BLUE
        assert subscription is not None
    finally:
        peer.destroy_node()
        node.destroy_node()
