"""Existing serial_comm_node + real C++ adapter through a PTY, NEVER a physical port."""
import os
from pathlib import Path
import signal
import struct
import subprocess
import time

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Vector3Stamped
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, Trigger
from srm_aim_comm.protocol import unpack_upper_frame


def main():
    master, slave = os.openpty()
    os.set_blocking(master, False)
    port = os.ttyname(slave)
    rclpy.init()
    peer = Node('manual_step_pty_test')
    connected, messages, wire = [], [], []
    subscriptions = [peer.create_subscription(Bool, '/srm_aim/gimbal_step/serial_connected', lambda m: connected.append(m.data), 10),
                     peer.create_subscription(DiagnosticArray, '/srm_aim/gimbal_step/debug', messages.append, 50)]
    step = peer.create_publisher(Vector3Stamped, '/srm_aim/gimbal_step/relative_step', 1)
    arm = peer.create_client(SetBool, '/srm_aim/gimbal_step/enable')
    stop = peer.create_client(Trigger, '/srm_aim/gimbal_step/stop')
    remaps = []
    for source, target in [('cmd_yaw', 'cmd_yaw'), ('cmd_pitch', 'cmd_pitch'),
                           ('serial_connected', 'serial_connected'), ('feedback_valid', 'feedback_valid'),
                           ('gimbal_state_stamped', 'feedback')]:
        remaps += ['-r', f'/srm_aim/{source}:=/srm_aim/gimbal_step/{target}']
    serial = subprocess.Popen(['python3', '-m', 'srm_aim_comm.serial_comm_node', '--ros-args',
                               '-p', 'enable_serial:=true', '-p', 'enable_tx:=true', '-p', 'port:='+port,
                               '-p', 'command_timeout_sec:=0.05', *remaps], start_new_session=True)
    exe = Path(get_package_prefix('srm_gimbal_follower'))/'lib/srm_gimbal_follower/manual_step_node'
    adapter = None
    buffer = bytearray()
    yaw = pitch = goal_yaw = goal_pitch = 0.
    rx = True
    last_lower = 0.

    def spawn(enabled):
        return subprocess.Popen([str(exe), '--ros-args', '-p', 'allow_hardware_commands:='+str(enabled).lower(),
                                 '-p', 'mechanical_limits_verified:=true', '-p', 'absolute_target_protocol_verified:=true',
                                 '-p', 'yaw_min_deg:=-1.0', '-p', 'yaw_max_deg:=1.0',
                                 '-p', 'pitch_min_deg:=-1.0', '-p', 'pitch_max_deg:=1.0'], start_new_session=True)

    def data():
        return {v.key: v.value for v in messages[-1].status[0].values} if messages else {}

    def pump(duration=.02):
        nonlocal yaw, pitch, goal_yaw, goal_pitch, last_lower
        end = time.monotonic()+duration
        while time.monotonic() < end:
            assert serial.poll() is None and adapter.poll() is None
            if rx and connected and connected[-1] and time.monotonic()-last_lower >= .02:
                yaw += .4*(goal_yaw-yaw)
                pitch += .4*(goal_pitch-pitch)
                os.write(master, struct.pack('<Hhfff i i h f', 28, 1, yaw, pitch, 0., 0, 0, 2, 22.))
                last_lower = time.monotonic()
            try:
                buffer.extend(os.read(master, 65536))
            except BlockingIOError:
                pass
            while len(buffer) >= 18:
                assert buffer[:2] == b'\x10\x00', 'unexpected protocol bytes'
                frame = bytes(buffer[:18]); del buffer[:18]
                y, p, fire = unpack_upper_frame(frame)
                assert fire == 0 and -1 <= y <= 1 and -1 <= p <= 1
                wire.append((y, p, fire))
                goal_yaw, goal_pitch = y, p
            rclpy.spin_once(peer, timeout_sec=.003)

    def wait(predicate):
        end = time.monotonic()+12
        while not predicate() and time.monotonic() < end:
            pump()
        assert predicate(), data()

    def enable():
        req = SetBool.Request(); req.data = True
        future = arm.call_async(req); wait(future.done)
        return future.result().success

    def send(dy, dp):
        m = Vector3Stamped(); m.header.stamp = peer.get_clock().now().to_msg()
        m.header.frame_id = 'relative_feedback_degrees'; m.vector.x, m.vector.y = dy, dp
        step.publish(m)

    try:
        adapter = spawn(False)
        wait(lambda: data().get('ready') == 'true' and arm.service_is_ready() and step.get_subscription_count())
        assert not enable()
        pump(.2); assert not wire
        adapter.send_signal(signal.SIGINT); adapter.wait(timeout=5)
        messages.clear()
        adapter = spawn(True)
        wait(lambda: data().get('ready') == 'true')
        assert enable()
        pump(.1); assert not wire
        send(.2, -.1)
        wait(lambda: len(wire)>5 and abs(yaw-.2)<.05 and abs(pitch+.1)<.05)
        future = stop.call_async(Trigger.Request()); wait(future.done)
        assert future.result().success
        pump(.15); count = len(wire); pump(.15); assert len(wire) == count
        pump(.5); assert enable()
        send(1.1, 0.)
        wait(lambda: data().get('armed') == 'false')
        assert len(wire) == count
        pump(.4); assert enable()
        send(.2, 0.)
        wait(lambda: len(wire)>count+1)
        rx = False
        pump(.4); count = len(wire); pump(.2)
        assert len(wire) == count and data().get('armed') == 'false'
        os.close(master); master = None
        end = time.monotonic()+5
        while time.monotonic() < end and connected[-1]:
            rclpy.spin_once(peer, timeout_sec=.02)
        assert not connected[-1] and data().get('armed') == 'false'
        assert subscriptions and wire
        print(f'PASS PTY: {len(wire)} actual 18-byte frames, all fire=0, bounds respected')
        print('PASS default disabled, arm alone silent, manual smooth step, explicit stop/cache drain, oversize rejection and feedback loss')
    finally:
        for proc in (adapter, serial):
            if proc is not None and proc.poll() is None:
                os.killpg(proc.pid, signal.SIGINT)
                try: proc.wait(timeout=5)
                except subprocess.TimeoutExpired: proc.kill(); proc.wait()
        peer.destroy_node(); rclpy.shutdown()
        if master is not None: os.close(master)
        os.close(slave)


if __name__ == '__main__':
    os.environ.setdefault('ROS_DOMAIN_ID', '87')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    main()
