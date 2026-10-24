"""Offline matching, drawing and ROS lifecycle tests; no camera/model required."""

from copy import deepcopy
from unittest.mock import Mock

import cv2
import numpy as np
import pytest
import rclpy
from cv_bridge import CvBridge
from rclpy.qos import ReliabilityPolicy
from sensor_msgs.msg import Image

from srm_aim_interfaces.msg import ArmorDetection, ArmorDetectionArray, Keypoint2D
from srm_aim_comm import yolo_visualization as drawing
from srm_aim_comm.yolo_visualization import DetectionRate, ExactFrameCache, draw_frame
from srm_aim_comm.yolo_visualizer_node import YoloVisualizerNode


def image(index=1, width=640, height=480, value=0):
    message = CvBridge().cv2_to_imgmsg(
        np.full((height, width, 3), value, np.uint8), 'bgr8')
    message.header.stamp.sec = 123
    message.header.stamp.nanosec = index
    message.header.frame_id = 'camera'
    return message


def detection(source, count=0):
    message = ArmorDetectionArray()
    message.header = deepcopy(source.header)
    message.image_width, message.image_height = source.width, source.height
    for index in range(count):
        x = 180.0 + index * 200
        message.detections.append(ArmorDetection(
            color=ArmorDetection.COLOR_BLUE if index == 0 else ArmorDetection.COLOR_RED,
            confidence=0.875,
            lt=Keypoint2D(x=x, y=120.0), lb=Keypoint2D(x=x, y=250.0),
            rb=Keypoint2D(x=x + 100, y=250.0), rt=Keypoint2D(x=x + 100, y=120.0)))
    return message


@pytest.mark.parametrize('detection_first', [False, True])
def test_exact_matching_both_arrival_orders(detection_first):
    cache = ExactFrameCache()
    source = image()
    result = detection(source, 2)
    if detection_first:
        assert cache.add_detection(result, 10.0) is None
        pair = cache.add_image(source, 10.1)
    else:
        assert cache.add_image(source, 10.0) is None
        pair = cache.add_detection(result, 10.1)
    assert pair[0] is source and pair[1] is result
    assert not cache.images and not cache.detections and cache.image_bytes == 0


@pytest.mark.parametrize('mismatch', ['stamp', 'frame_id'])
def test_never_uses_nearest_or_latest_image(mismatch):
    cache = ExactFrameCache()
    source = image()
    result = detection(source)
    if mismatch == 'stamp':
        result.header.stamp.nanosec += 1
    else:
        result.header.frame_id = 'another_camera'
    cache.add_image(source, 1)
    assert cache.add_detection(result, 1.1) is None


def test_size_mismatch_and_expired_image_are_dropped():
    source = image()
    cache = ExactFrameCache(ttl=0.1)
    cache.add_image(source, 0)
    assert cache.add_detection(detection(source), 0.2) is None
    assert not cache.images
    result = detection(source)
    result.image_width += 1
    cache = ExactFrameCache()
    cache.add_image(source, 0)
    assert cache.add_detection(result, 0.1) is None
    assert cache.image_bytes == 0


def test_count_byte_limits_and_idle_expiration():
    source = image(width=8, height=8)
    cache = ExactFrameCache(image_limit=2, detection_limit=2,
                            byte_limit=len(source.data) * 2, ttl=1)
    for index in range(1000):
        source = image(index, width=8, height=8)
        cache.add_image(source, 0.01)
        cache.add_detection(detection(image(index + 10000, width=8, height=8)), 0.02)
        assert len(cache.images) <= 2 and len(cache.detections) <= 2
        assert cache.image_bytes <= 384
    cache.prune(2)
    assert not cache.images and not cache.detections and cache.image_bytes == 0


def test_byte_limit_independently_evicts_and_rejects_oversize():
    source = image(width=8, height=8)
    cache = ExactFrameCache(image_limit=10, byte_limit=len(source.data))
    cache.add_image(source, 0)
    cache.add_image(image(2, width=8, height=8), 0.1)
    assert len(cache.images) == 1 and cache.evicted == 1
    assert cache.add_image(image(3, width=16, height=16), 0.2) is None
    assert len(cache.images) == 1


def test_ambiguous_duplicate_image_and_duplicate_result_do_not_overlay():
    cache = ExactFrameCache()
    source = image()
    cache.add_image(source, 0)
    assert cache.add_image(source, 0.1) is None
    assert cache.add_detection(detection(source), 0.2) is None
    assert not cache.images
    source = image(2)
    cache.add_image(source, 0.3)
    assert cache.add_detection(detection(source), 0.4) is not None
    assert cache.add_detection(detection(source), 0.5) is None


def test_receipt_ttl_does_not_assume_wall_clock_source_timestamps():
    source = image()
    cache = ExactFrameCache()
    cache.add_image(source, 1000000)
    assert cache.add_detection(detection(source), 1000000.1) is not None


def test_duplicate_history_is_bounded_under_continuous_matching():
    cache = ExactFrameCache(image_limit=2, detection_limit=3)
    for index in range(1000):
        source = image(index, width=8, height=8)
        cache.add_image(source, 0)
        assert cache.add_detection(detection(source), 0.1) is not None
        assert len(cache.blocked) <= 5
        assert cache.image_bytes == 0
    cache.prune(2)
    assert not cache.blocked


@pytest.mark.parametrize('kwargs', [
    {'image_limit': 0}, {'detection_limit': 0}, {'byte_limit': 0},
    {'ttl': float('nan')}, {'ttl': -1}])
def test_invalid_cache_configuration(kwargs):
    with pytest.raises(ValueError):
        ExactFrameCache(**kwargs)


def test_invalid_stamp_rejected():
    source = image()
    source.header.stamp.nanosec = 1000000000
    with pytest.raises(ValueError):
        ExactFrameCache().add_image(source, 0)


def test_detection_fps_is_received_message_rate_and_expires():
    rate = DetectionRate()
    for index in range(18):
        rate.add(index / 17)
    assert rate.fps(1) == pytest.approx(17)
    assert rate.fps(4) == 0


@pytest.fixture
def text_calls(monkeypatch):
    calls = []
    original = drawing._text

    def record(frame, text, *args, **kwargs):
        calls.append(text)
        return original(frame, text, *args, **kwargs)
    monkeypatch.setattr(drawing, '_text', record)
    return calls


def test_draws_every_target_labels_and_green_edges_without_modifying_image(text_calls):
    source = np.zeros((480, 640, 3), np.uint8)
    result = detection(image(), 2)
    canvas = draw_frame(source, result, 17, 0.02)
    assert not np.any(source)
    assert np.array_equal(canvas[200, 180], [0, 255, 0])
    assert np.array_equal(canvas[200, 380], [0, 255, 0])
    for name in ('LT', 'LB', 'RB', 'RT'):
        assert text_calls.count(name) == 2
    assert 'Blue 0.875' in text_calls and 'Red 0.875' in text_calls
    assert any('Det FPS (rx): 17.0  Targets: 2' in t for t in text_calls)
    assert any('123.000000001' in t and 'frame_id=camera' in t for t in text_calls)


def test_no_target_and_stale_views_remove_previous_overlays(text_calls):
    source = np.zeros((480, 640, 3), np.uint8)
    empty = draw_frame(source, detection(image()), 0, 0)
    assert 'No Target' in text_calls
    assert not np.any(empty[100:])
    stale = draw_frame(source, detection(image(), 2), 0, 1.0, stale=True)
    assert not np.any(stale[100:])
    assert any('STALE' in t for t in text_calls)


@pytest.mark.parametrize('failure', ['nan', 'bounds', 'degenerate', 'confidence'])
def test_invalid_instances_do_not_break_rendering(failure, text_calls):
    result = detection(image(), 2)
    first = result.detections[0]
    if failure == 'nan':
        first.lt.x = float('nan')
    elif failure == 'bounds':
        first.lt.x = 640.0
    elif failure == 'degenerate':
        first.lb = deepcopy(first.lt)
    else:
        first.confidence = float('inf')
    draw_frame(np.zeros((480, 640, 3), np.uint8), result, 17, 0)
    assert any('Targets: 1  Invalid: 1' in t for t in text_calls)


@pytest.fixture
def node(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '75')
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    monkeypatch.setenv('DISPLAY', ':mock')
    for name in ('namedWindow', 'resizeWindow', 'imshow', 'destroyWindow'):
        monkeypatch.setattr(cv2, name, Mock())
    monkeypatch.setattr(cv2, 'waitKey', Mock(return_value=-1))
    monkeypatch.setattr(cv2, 'getWindowProperty', Mock(return_value=1))
    rclpy.init()
    value = YoloVisualizerNode()
    yield value
    value.destroy_node()
    rclpy.shutdown()


def test_subscriptions_accept_best_effort_and_reliable_writers(node):
    assert node.image_sub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
    assert node.detection_sub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
    assert node.image_sub.msg_type is Image
    assert node.detection_sub.msg_type is ArmorDetectionArray


def test_callback_uses_old_exact_image_instead_of_latest(node):
    old, latest = image(1, value=10), image(2, value=90)
    node.image_callback(old)
    node.image_callback(latest)
    node.detection_callback(detection(old, 2))
    node.update_display()
    assert node._last_detection.header == old.header
    assert np.all(node._last_image == 10)
    assert cv2.imshow.called


def test_unmatched_result_does_not_replace_current_frame(node):
    source = image()
    node.image_callback(source)
    node.detection_callback(detection(image(2)))
    node.update_display()
    assert node._last_image is None


def test_out_of_order_matches_do_not_rewind_display(node):
    newer, older = image(2), image(1)
    node.image_callback(newer)
    node.detection_callback(detection(newer))
    node.image_callback(older)
    node.detection_callback(detection(older))
    node.update_display()
    assert node._last_detection.header == newer.header


def test_q_requests_clean_exit(node):
    cv2.waitKey.return_value = ord('q')
    node.update_display()
    assert node.stop_requested


def test_window_close_requests_exit(node):
    cv2.getWindowProperty.return_value = 0
    node.update_display()
    assert node.stop_requested


def test_backend_without_visibility_query_does_not_exit_at_startup(node):
    cv2.getWindowProperty.return_value = -1
    node.update_display()
    assert not node.stop_requested


def test_missing_display_fails_with_clear_error(monkeypatch):
    monkeypatch.setenv('ROS_DOMAIN_ID', '75')
    monkeypatch.delenv('DISPLAY', raising=False)
    monkeypatch.delenv('WAYLAND_DISPLAY', raising=False)
    rclpy.init()
    try:
        with pytest.raises(RuntimeError, match='display unavailable'):
            YoloVisualizerNode()
    finally:
        rclpy.shutdown()
