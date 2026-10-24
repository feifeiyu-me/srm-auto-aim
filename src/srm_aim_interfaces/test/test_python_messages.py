from rclpy.serialization import deserialize_message, serialize_message

from srm_aim_interfaces.msg import (
    ArmorDetection,
    ArmorDetectionArray,
    Keypoint2D,
)


def test_multitarget_serialization_preserves_source_header_and_corner_order():
    frame = ArmorDetectionArray()
    frame.header.stamp.sec = 123
    frame.header.stamp.nanosec = 456789012
    frame.header.frame_id = 'camera_optical_frame'
    frame.image_width = 1440
    frame.image_height = 1080
    for color, offset in [(ArmorDetection.COLOR_RED, 0.0),
                          (ArmorDetection.COLOR_BLUE, 500.0)]:
        frame.detections.append(ArmorDetection(
            color=color,
            confidence=0.875,
            lt=Keypoint2D(x=100.25 + offset, y=200.5),
            lb=Keypoint2D(x=101.0 + offset, y=300.0),
            rb=Keypoint2D(x=400.0 + offset, y=301.0),
            rt=Keypoint2D(x=399.0 + offset, y=201.0),
        ))

    decoded = deserialize_message(serialize_message(frame), ArmorDetectionArray)

    assert decoded == frame
    assert len(decoded.detections) == 2
    assert decoded.detections[0].lt.x == 100.25
    assert decoded.detections[0].lb.y == 300.0
    assert decoded.detections[0].rb.x == 400.0
    assert decoded.detections[0].rt.y == 201.0
    assert decoded.detections[1].color == ArmorDetection.COLOR_BLUE


def test_no_detections_is_an_empty_stamped_frame():
    frame = ArmorDetectionArray()
    frame.header.stamp.sec = 124
    frame.header.frame_id = 'camera'
    frame.image_width = 1440
    frame.image_height = 1080

    decoded = deserialize_message(serialize_message(frame), ArmorDetectionArray)

    assert decoded == frame
    assert list(decoded.detections) == []


def test_unclassified_armor_uses_unknown_color():
    assert ArmorDetection().color == ArmorDetection.COLOR_UNKNOWN
