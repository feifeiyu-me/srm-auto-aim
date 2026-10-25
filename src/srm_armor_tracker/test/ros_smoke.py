"""Separate real C++ tracker process and Python ROS peer; synthetic positions only."""
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from srm_aim_interfaces.msg import ArmorLocation, ArmorLocationArray, ArmorTrackArray


def main():
    rclpy.init()
    peer = Node('tracker_test_peer')
    exe = Path(get_package_prefix('srm_armor_tracker')) / 'lib/srm_armor_tracker/armor_tracker_node'
    config = Path(get_package_share_directory('srm_armor_tracker')) / 'config/tracker.yaml'
    process = subprocess.Popen([str(exe), '--ros-args', '--params-file', str(config),
                                '-p', 'input_timeout_sec:=1.0'],
                               start_new_session=True)
    received, diagnostics = [], []
    publisher = peer.create_publisher(ArmorLocationArray, '/srm_aim/armor_locations', 5)
    subscription = peer.create_subscription(ArmorTrackArray, '/srm_aim/armor_tracks', received.append, 100)
    debug = peer.create_subscription(DiagnosticArray, '/srm_aim/armor_tracker/debug', diagnostics.append, 100)

    def pump(duration=.03):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            assert process.poll() is None, 'tracker exited'
            rclpy.spin_once(peer, timeout_sec=.005)

    def wait(predicate):
        end = time.monotonic() + 10
        while not predicate() and time.monotonic() < end:
            pump()
        assert predicate(), 'ROS result timeout'

    def message(t, xs=(), colors=None):
        m = ArmorLocationArray()
        stamp = round(t * 1e9)
        m.header.stamp.sec, m.header.stamp.nanosec = divmod(stamp, 10**9)
        m.header.frame_id = 'source_camera'
        m.optical_frame_id = 'camera_optical_frame'
        m.image_width, m.image_height = 1440, 1080
        for i, x in enumerate(xs):
            loc = ArmorLocation()
            loc.input_index = i
            loc.pose_valid = loc.orientation_valid = loc.position_valid = True
            loc.pose.position.x, loc.pose.position.z = float(x), 3.0
            loc.position.x, loc.position.z = float(x), 3.0
            loc.detection.color = colors[i] if colors else 2
            loc.detection.confidence = .85
            loc.digit_score = -1.0
            m.locations.append(loc)
        return m

    def send(m):
        start = len(received)
        publisher.publish(m)
        wait(lambda: len(received) > start)
        out = received[-1]
        assert out.header == m.header
        assert out.image_width == m.image_width and out.image_height == m.image_height
        assert out.tracking_frame_id == 'camera_optical_frame'
        assert not out.self_motion_compensated
        for track in out.tracks:
            assert not track.digit_check_enabled and not track.digit_verified and track.digit == 0
            assert all(math.isfinite(v) for v in track.covariance)
            for i in range(6):
                for j in range(6):
                    assert abs(track.covariance[6*i+j]-track.covariance[6*j+i]) < 1e-8
        return out

    def tracks(out):
        return {t.track_id: t for t in out.tracks if t.state != t.DELETED}

    try:
        wait(lambda: publisher.get_subscription_count() == 1)
        pump(.3)
        # Two targets, changing source array order, one moving and one stationary.
        out = send(message(10, [-.5, .5], [1, 2]))
        red, blue = out.tracks[0].track_id, out.tracks[1].track_id
        assert not out.tracks[0].velocity_valid and not out.tracks[0].prediction_valid
        for k in range(1, 31):
            x = -.5 + k*.0625*.3
            msg = message(10+k*.0625, [.5, x], [2, 1])
            if k % 2:
                # Alternate strict full poses and independent positions with legacy zeros.
                for loc in msg.locations:
                    loc.pose_valid = loc.orientation_valid = False
                    loc.ambiguous = True
                    loc.reason = 'ambiguous_ippe'
                    loc.pose.position.x = loc.pose.position.y = loc.pose.position.z = 0.0
            out = send(msg)
            tt = tracks(out)
            assert set(tt) == {red, blue}
            assert tt[red].input_index == 1 and tt[blue].input_index == 0
        moving = tracks(out)[red]
        assert moving.velocity_valid and abs(moving.velocity.x-.3) < .02
        assert abs(moving.predicted_position.x - (moving.position.x+.1*.3)) < .005
        assert abs(tracks(out)[blue].velocity.x) < 1e-8
        assert moving.prediction_stamp != moving.state_stamp
        print('PASS full/position-only mixed tracking, stable IDs, reordered multi-target, velocity/covariance/prediction, Header/unverified')

        # Short disappearance retains IDs; invalid observations cannot update.
        out = send(message(11.9375))
        assert all(t.state == t.COASTING for t in out.tracks)
        bad = message(12, [.5, .1, float('nan')], [2, 1, 2])
        bad.locations[0].position_valid = False  # Valid legacy Pose must not be used.
        bad.locations[1].position_valid = False
        bad.locations[1].pose_valid = bad.locations[1].orientation_valid = False
        bad.locations[1].position.x = bad.locations[1].position.y = bad.locations[1].position.z = 0.0
        bad.locations[1].pose.position.x = bad.locations[1].pose.position.y = bad.locations[1].pose.position.z = 0.0
        out = send(bad)
        assert all(not t.matched_this_frame for t in out.tracks)
        out = send(message(12.0625, [.11875, .5], [1, 2]))
        assert set(tracks(out)) == {red, blue}
        # Absolute-distance rejection does not drag an existing track.
        out = send(message(12.125, [20], [1]))
        assert abs(tracks(out)[red].position.x) < 1
        print('PASS empty, short loss/reacquisition, invalid PnP/NaN and abnormal jump')

        # Duplicates and small backward jumps are rejected, rather than applied twice.
        before = len(received)
        publisher.publish(message(12.125, [0]))
        publisher.publish(message(12.12, [0]))
        pump(.15)
        assert len(received) == before
        session = out.session_id
        out = send(message(1, [-.3, .3]))
        assert out.reason == 'clock_reset' and out.session_id != session
        left, right = out.tracks[0].track_id, out.tracks[1].track_id
        ambiguity = False
        for k in range(1, 21):
            x = -.3 + k*.03
            out = send(message(1+k*.0625, [x, -x]))
            assert set(tracks(out)) == {left, right}
            ambiguity |= any(t.reason == 'association_ambiguous' for t in out.tracks)
        assert ambiguity and tracks(out)[left].position.x > 0 and tracks(out)[right].position.x < 0
        print('PASS duplicate/out-of-order/clock reset and conservative same-color crossing')

        # Wrong coordinate frame clears all state; source/size/gap changes reset session.
        wrong = message(2.3125, [0])
        wrong.optical_frame_id = 'world'
        out = send(wrong)
        assert not out.tracks and out.reason == 'invalid_coordinate_metadata'
        out = send(message(2.375, [0]))
        changed = message(2.4375, [0]); changed.image_width = 640
        out = send(changed)
        assert out.reason == 'source_changed'
        out = send(message(5, [0]))
        # Source size also changed, so source reset takes precedence over long gap.
        assert out.reason == 'source_changed'
        out = send(message(7, [0]))
        assert out.reason == 'long_gap_reset'
        out = send(message(7.4))
        assert out.tracks[0].state == out.tracks[0].DELETED
        out = send(message(7.45))
        assert not out.tracks
        out = send(message(7.5, [0]))
        start = len(received)
        wait(lambda: len(received) > start)
        assert received[-1].reason == 'input_timeout'
        assert received[-1].tracks[0].state == received[-1].tracks[0].DELETED
        after = len(received); pump(.2)
        assert len(received) == after
        assert any(s.message == 'rejected_timestamp' for d in diagnostics for s in d.status)
        assert subscription and debug
        forbidden = {'/srm_aim/cmd_yaw', '/srm_aim/cmd_pitch', '/srm_aim/cmd_fire',
                     '/srm_aim/yaw_error', '/srm_aim/pitch_error'}
        topics = {name for name, _ in peer.get_publisher_names_and_types_by_node('armor_tracker_node', '/')}
        assert not topics & forbidden
        print('PASS coordinate/size/long-gap reset, deletion, monotonic input timeout, diagnostics, no control publishers')
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
    os.environ.setdefault('ROS_DOMAIN_ID', '81')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    main()
