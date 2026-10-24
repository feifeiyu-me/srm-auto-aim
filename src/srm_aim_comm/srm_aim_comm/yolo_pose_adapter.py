"""Translate Ultralytics pose results into the shared image-space interface."""

from copy import deepcopy

import numpy as np

from srm_aim_interfaces.msg import ArmorDetection, ArmorDetectionArray, Keypoint2D


def validate_model(model):
    """Reject incompatible models instead of interpreting human poses as armor."""
    if model.task != 'pose':
        raise ValueError('Expected a pose model')
    shape = getattr(model.model, 'kpt_shape', None)
    if shape is None:
        shape = getattr(model.model, 'yaml', {}).get('kpt_shape')
    if tuple(shape or ()) != (4, 3):
        raise ValueError(f'Expected kpt_shape=[4, 3], got {shape}')
    names = model.names
    if isinstance(names, (list, tuple)):
        names = dict(enumerate(names))
    if names != {0: 'blue', 1: 'red'}:
        raise ValueError(f'Expected classes {{0: blue, 1: red}}, got {names}')


def _numpy(value):
    if hasattr(value, 'detach'):
        value = value.detach()
    if hasattr(value, 'cpu'):
        value = value.cpu()
    if hasattr(value, 'numpy'):
        value = value.numpy()
    return np.asarray(value)


def valid_quad(points, width, height, min_area, min_edge, max_edge_ratio):
    """Check LT/LB/RB/RT winding, convexity, size, and original-image bounds."""
    if not np.isfinite(points).all():
        return False
    if (np.any(points < 0) or np.any(points[:, 0] >= width)
            or np.any(points[:, 1] >= height)):
        return False
    edges = np.roll(points, -1, axis=0) - points
    lengths = np.linalg.norm(edges, axis=1)
    if lengths.min() < min_edge or lengths.max() / lengths.min() > max_edge_ratio:
        return False
    following = np.roll(edges, -1, axis=0)
    cross = edges[:, 0] * following[:, 1] - edges[:, 1] * following[:, 0]
    if not np.all(cross < -1e-6):
        return False
    area = abs(np.sum(points[:, 0] * np.roll(points[:, 1], -1)
                      - points[:, 1] * np.roll(points[:, 0], -1))) / 2
    return area >= min_area


def result_to_message(result, header, width, height, *, confidence=0.25,
                      keypoint_confidence=0.5, corner_indices=(0, 1, 2, 3),
                      min_area=16.0, min_edge=2.0, max_edge_ratio=20.0):
    """Convert one image result; malformed output raises, invalid instances skip."""
    if tuple(result.orig_shape) != (height, width):
        raise ValueError('Result orig_shape does not match the source image')
    message = ArmorDetectionArray()
    message.header = deepcopy(header)
    message.image_width = width
    message.image_height = height
    if result.boxes is None:
        raise ValueError('Pose result has no boxes object')
    scores = _numpy(result.boxes.conf)
    classes = _numpy(result.boxes.cls)
    if scores.ndim != 1 or classes.shape != scores.shape:
        raise ValueError('Invalid detection confidence/class tensor shapes')
    count = len(scores)
    if count == 0:
        return message
    if result.keypoints is None or result.keypoints.conf is None:
        raise ValueError('Pose result is missing keypoints or their confidence')
    # Validate the float32 values that will actually be published on the wire.
    points = _numpy(result.keypoints.xy).astype(np.float32)
    point_scores = _numpy(result.keypoints.conf)
    if points.shape != (count, 4, 2) or point_scores.shape != (count, 4):
        raise ValueError('Expected xy[N,4,2] and keypoint confidence[N,4]')
    for score, label, corners, visibility in zip(scores, classes, points, point_scores):
        if (not np.isfinite(score) or not confidence <= score <= 1.0
                or not np.isfinite(label) or label not in (0, 1)):
            continue
        if (not np.isfinite(visibility).all()
                or np.any(visibility < keypoint_confidence)
                or np.any(visibility > 1.0)):
            continue
        corners = corners[list(corner_indices)]
        if not valid_quad(corners, width, height, min_area, min_edge, max_edge_ratio):
            continue
        detection = ArmorDetection()
        detection.color = (ArmorDetection.COLOR_BLUE if label == 0
                           else ArmorDetection.COLOR_RED)
        detection.confidence = float(score)
        for field, point in zip(('lt', 'lb', 'rb', 'rt'), corners):
            setattr(detection, field, Keypoint2D(x=float(point[0]), y=float(point[1])))
        message.detections.append(detection)
    return message
