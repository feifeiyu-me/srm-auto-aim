"""Separate real C++ ROS process, synthetic ArmorTrackArray; no hardware nodes."""
from pathlib import Path
import math
import os
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from diagnostic_msgs.msg import DiagnosticArray
from rclpy.node import Node
from srm_aim_interfaces.msg import ArmorTrack, ArmorTrackArray


def run(reference=False):
    rclpy.init()
    peer = Node('visual_follower_test_peer')
    exe = Path(get_package_prefix('srm_visual_follower'))/'lib/srm_visual_follower/visual_follower_node'
    config = Path(get_package_share_directory('srm_visual_follower'))/'config/follower.yaml'
    args = [str(exe), '--ros-args', '--params-file', str(config),
            '-p', 'input_timeout_sec:=0.6', '-p', 'target_timeout_sec:=0.25']
    if reference:
        args += ['-p', 'extrinsics_valid:=true', '-p', 'reference_frame:=test_reference_flu',
                 '-p', 'quaternion_xyzw:=[-0.5,0.5,-0.5,0.5]']
    process = subprocess.Popen(args, start_new_session=True)
    received = []
    publisher = peer.create_publisher(ArmorTrackArray, '/srm_aim/armor_tracks', 5)
    subscription = peer.create_subscription(DiagnosticArray, '/srm_aim/visual_follower/debug', received.append, 50)

    def pump(duration=.02):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            assert process.poll() is None, 'follower exited'
            rclpy.spin_once(peer, timeout_sec=.005)

    def wait(predicate):
        end = time.monotonic()+10
        while not predicate() and time.monotonic() < end:
            pump()
        assert predicate(), 'ROS result timeout'

    def values(message):
        assert len(message.status) == 1
        return {v.key: v.value for v in message.status[0].values}

    def frame(items, session='synthetic_session'):
        m = ArmorTrackArray()
        m.header.stamp = peer.get_clock().now().to_msg()
        m.header.frame_id = 'source_camera'
        m.session_id = session
        m.tracking_frame_id = 'camera_optical_frame'
        m.image_width, m.image_height = 1440, 1080
        for track_id, x, y, z, prediction, valid in items:
            t = ArmorTrack(track_id=track_id, state=ArmorTrack.CONFIRMED, state_valid=valid)
            t.state_stamp = t.last_observation_stamp = m.header.stamp
            t.position.x, t.position.y, t.position.z = float(x), float(y), float(z)
            if prediction is not None:
                t.prediction_valid = True
                ns = peer.get_clock().now().nanoseconds+150000000
                t.prediction_stamp.sec, t.prediction_stamp.nanosec = divmod(ns, 10**9)
                t.predicted_position.x, t.predicted_position.y, t.predicted_position.z = prediction
            m.tracks.append(t)
        return m

    def send(m):
        before = len(received)
        publisher.publish(m)
        wait(lambda: any(v.header == m.header for v in received[before:]))
        result = next(v for v in received[before:] if v.header == m.header)
        data = values(result)
        assert data['hardware_command_valid'] == 'false'
        assert data['mode'] == 'diagnostics_only'
        assert data['source_stamp_sec'] == str(m.header.stamp.sec)
        assert data['source_stamp_nanosec'] == str(m.header.stamp.nanosec)
        if not reference:
            assert data['reference_los_valid'] == 'false'
        return result, data

    try:
        wait(lambda: publisher.get_subscription_count() == 1)
        pump(.2)
        if reference:
            _, data = send(frame([(7, 1, -1, 2, None, True)]))
            assert data['reference_los_valid'] == 'true'
            assert abs(float(data['reference_position_x_m'])-2) < 1e-9
            assert abs(float(data['reference_position_y_m'])+1) < 1e-9
            assert abs(float(data['reference_pitch_rad'])-math.atan2(1, math.sqrt(5))) < 1e-9
            assert data['hardware_command_valid'] == 'false'
            print('PASS explicit extrinsics produce reference LOS only; hardware commands remain invalid')
        else:
            first = frame([(9, 1, 0, 3, None, True), (2, 0, 0, 3, (.3, -.3, 3.), True)])
            _, data = send(first)
            assert data['track_id'] == '2' and data['position_source'] == 'predicted'
            assert float(data['camera_yaw_rad']) > 0 and float(data['camera_pitch_rad']) > 0
            _, data = send(frame([(9, 0, 0, 3, None, True), (2, 1, 0, 3, None, True)]))
            assert data['track_id'] == '2' and data['position_source'] == 'current'
            _, data = send(frame([(2, 1, 0, 3, None, False), (9, 0, 0, 3, None, True)]))
            assert data['track_id'] == '9'
            _, data = send(frame([(2, 0, 0, 3, None, True), (9, 1, 0, 3, None, True)], 'session_b'))
            assert data['track_id'] == '2'
            _, data = send(frame([(2, 0, 0, 3, (float('nan'), 0., 3.), True)], 'session_b'))
            assert data['target_valid'] == 'true' and data['position_source'] == 'current'
            _, data = send(frame([(2, 0, 0, -1, None, True)], 'session_b'))
            assert data['target_valid'] == 'false'
            last = frame([(4, 0, 0, 3, None, True)], 'session_b')
            send(last)
            pump(.02)
            start = len(received)
            publisher.publish(last)
            wait(lambda: any(v.status[0].message == 'duplicate_or_out_of_order' for v in received[start:]))
            _, data = send(frame([(4, 0, 0, 3, None, True)], 'session_b'))
            assert data['target_valid'] == 'true'
            wait(lambda: any(v.status[0].message == 'target_timeout_or_invalid_state' for v in received))
            wait(lambda: any(v.status[0].message == 'input_timeout' for v in received))
            assert values(received[-1])['target_valid'] == 'false'
            assert float(values(received[-1])['camera_yaw_rad']) == 0
            print('PASS prediction/current sources, sticky selection/session reset, invalids, duplicates and target/input timeout')
        names = {name for name, _ in peer.get_publisher_names_and_types_by_node('visual_follower_node', '/')}
        assert '/srm_aim/visual_follower/debug' in names
        assert not any(name.startswith('/srm_aim/cmd_') for name in names)
        assert not names & {'/srm_aim/yaw_error', '/srm_aim/pitch_error'}
        assert subscription
        print('PASS node graph contains diagnostic output and no control publishers')
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


def startup_rejections():
    exe = Path(get_package_prefix('srm_visual_follower'))/'lib/srm_visual_follower/visual_follower_node'
    for extra in [ ['-p', 'extrinsics_valid:=true'],
                   ['-r', '/srm_aim/visual_follower/debug:=/srm_aim/cmd_yaw'],
                   ['-r', '/srm_aim/visual_follower/debug:=/srm_aim/cmd_pitch'] ]:
        result = subprocess.run([str(exe), '--ros-args', *extra], capture_output=True, text=True, timeout=10)
        assert result.returncode == 1 and 'Follower failed:' in result.stderr
    print('PASS missing extrinsics and remapping diagnostics to command topics rejected at startup')


if __name__ == '__main__':
    os.environ.setdefault('ROS_DOMAIN_ID', '85')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    run(False)
    run(True)
    startup_rejections()
