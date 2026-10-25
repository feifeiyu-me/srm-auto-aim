"""Real launch, two C++ processes, ROS messages/services; synthetic plant only."""
import argparse
import csv
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from ament_index_python.packages import get_package_prefix
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Vector3Stamped
from rcl_interfaces.srv import SetParameters
from rclpy.node import Node
from rclpy.parameter import Parameter


def main(output_dir=None):
    rclpy.init()
    peer = Node('gimbal_simulation_test_peer')
    process = subprocess.Popen(['ros2', 'launch', 'srm_gimbal_follower', 'simulation.launch.py'], start_new_session=True)
    start = time.monotonic()
    start_ros_ns = peer.get_clock().now().nanoseconds
    feedback, diagnostics, phases = [], [], []
    subscriptions = [
        peer.create_subscription(Vector3Stamped, '/srm_aim/gimbal_follower/sim_state_stamped',
                                 lambda m: feedback.append((time.monotonic()-start, m)), 100),
        peer.create_subscription(DiagnosticArray, '/srm_aim/gimbal_follower/debug',
                                 lambda m: diagnostics.append((time.monotonic()-start, m)), 100)]
    client = peer.create_client(SetParameters, '/gimbal_follower_simulator/set_parameters')

    def pump(duration=.02):
        end = time.monotonic()+duration
        while time.monotonic() < end:
            assert process.poll() is None, 'simulation launch exited'
            rclpy.spin_once(peer, timeout_sec=.005)

    def wait(predicate, timeout=12):
        end = time.monotonic()+timeout
        while not predicate() and time.monotonic() < end:
            pump()
        assert predicate(), 'ROS simulation check timed out'

    def values(m):
        assert len(m.status) == 1 and m.status[0].name == 'srm_gimbal_follower/reference'
        return {v.key: v.value for v in m.status[0].values}

    def data():
        return values(diagnostics[-1][1]) if diagnostics else {}

    def valid():
        return data().get('simulation_reference_valid') == 'true'

    def position():
        for _, m in reversed(feedback):
            if math.isfinite(m.vector.x) and math.isfinite(m.vector.y):
                return m.vector.x, m.vector.y
        return float('inf'), float('inf')

    def near(yaw, pitch, tolerance=.25):
        a, b = position()
        return valid() and abs(a-yaw) < tolerance and abs(b-pitch) < tolerance

    def settled(yaw, pitch):
        if not near(yaw, pitch, .15) or len(feedback) < 12:
            return False
        recent = [m.vector for _, m in feedback[-12:]]
        if not all(math.isfinite(v.x) and math.isfinite(v.y) for v in recent):
            return False
        return (all(abs(v.x-yaw) < .15 and abs(v.y-pitch) < .15 for v in recent)
                and max(v.x for v in recent)-min(v.x for v in recent) < .05
                and max(v.y for v in recent)-min(v.y for v in recent) < .05)

    def set_params(**kwargs):
        request = SetParameters.Request()
        request.parameters = [Parameter(k, value=v).to_parameter_msg() for k, v in kwargs.items()]
        future = client.call_async(request)
        wait(future.done)
        assert all(v.successful for v in future.result().results)

    def phase(label):
        phases.append({'time_sec': time.monotonic()-start, 'name': label})

    def wait_reason(reason, since):
        wait(lambda: any(m.status[0].message == reason for _, m in diagnostics[since:]))

    try:
        wait(lambda: client.service_is_ready() and feedback and valid())
        phase('initial_two_axis_follow')
        wait(lambda: settled(-10, -6))
        initial = position()
        assert initial[0] < 0 and initial[1] < 0
        print('PASS both axes converge: right/up motion decreases yaw/pitch feedback', initial)

        phase('visual_loss')
        before = len(diagnostics)
        set_params(publish_visual=False)
        wait_reason('target_timeout', before)
        pump(.1)
        held = position()
        pump(.2)
        assert abs(position()[0]-held[0]) < .02 and abs(position()[1]-held[1]) < .02
        set_params(publish_visual=True)
        wait(valid)
        print('PASS target loss invalidates reference and simulated plant holds')

        phase('feedback_loss')
        before = len(diagnostics)
        set_params(publish_feedback=False)
        wait_reason('feedback_timeout', before)
        assert not valid()
        set_params(publish_feedback=True)
        wait(valid)
        phase('nonfinite_feedback')
        before = len(diagnostics)
        set_params(invalid_feedback=True)
        wait_reason('feedback_fault', before)
        assert not valid()
        set_params(invalid_feedback=False)
        wait(valid)
        print('PASS stale and nonfinite feedback stop updates; recovery waits for aligned data')

        phase('guarded_target_switch')
        before = len(diagnostics)
        set_params(target_id=2, target_yaw_flu_deg=8.0, target_pitch_up_deg=-4.0)
        wait_reason('target_switch_pending', before)
        wait(lambda: valid() and data().get('track_id') == '2')
        phase('reverse_two_axis_follow')
        wait(lambda: settled(8, 4))
        final = position()
        assert final[0] > 0 and final[1] > 0
        print('PASS target ID guard and opposite two-axis response', final)

        phase('invalid_visual')
        before = len(diagnostics)
        set_params(invalid_visual=True)
        wait_reason('visual_target_invalid', before)
        assert not valid()
        set_params(invalid_visual=False)
        wait(valid)

        contiguous, max_rate, max_step = 0, 0., 0.
        previous = None
        for _, m in diagnostics:
            d = values(m)
            assert d['mode'] == 'simulation'
            assert d['real_hardware_enabled'] == d['serial_tx_enabled'] == 'false'
            if d['simulation_reference_valid'] != 'true':
                assert d['hold_requested'] == 'true'
                previous = None
                continue
            assert d['position_source'] in ('current', 'predicted')
            y, p = float(d['yaw_reference_deg']), float(d['pitch_reference_deg'])
            assert -170 <= y <= 170 and -45 <= p <= 45
            if previous and int(d['sequence']) == int(previous['sequence'])+1 and d['track_id'] == previous['track_id']:
                dt = float(d['dt_sec'])
                step = max(abs(y-float(previous['yaw_reference_deg'])), abs(p-float(previous['pitch_reference_deg'])))
                assert step <= float(d['max_step_deg'])+1e-7
                assert step <= float(d['max_rate_deg_sec'])*dt+1e-7
                max_rate, max_step = max(max_rate, step/dt), max(max_step, step)
                contiguous += 1
            previous = d
        assert contiguous > 20
        for node in ('gimbal_follower_node', 'gimbal_follower_simulator'):
            pubs = {name for name, _ in peer.get_publisher_names_and_types_by_node(node, '/')}
            assert not any(name.startswith('/srm_aim/cmd_') for name in pubs)
            assert not pubs & {'/srm_aim/yaw_error', '/srm_aim/pitch_error', '/srm_aim/gimbal_state_stamped'}
        assert subscriptions
        summary = {'initial_feedback_deg': initial, 'reversed_feedback_deg': final,
                   'contiguous_reference_pairs_checked': contiguous, 'max_observed_reference_rate_deg_sec': max_rate,
                   'max_observed_reference_step_deg': max_step, 'phases': phases,
                   'scope': 'synthetic plant only; not real gimbal performance'}
        if output_dir:
            directory = Path(output_dir)
            directory.mkdir(parents=True, exist_ok=True)
            (directory/'summary.json').write_text(json.dumps(summary, indent=2), encoding='utf-8')
            with (directory/'response.csv').open('w', newline='', encoding='utf-8') as stream:
                writer = csv.writer(stream)
                writer.writerow(['time_sec', 'source_time_sec', 'stamp_sec', 'stamp_nanosec',
                                 'yaw_feedback_deg', 'pitch_feedback_deg'])
                for t, m in feedback:
                    stamp_ns = m.header.stamp.sec*10**9+m.header.stamp.nanosec
                    writer.writerow([t, (stamp_ns-start_ros_ns)*1e-9, m.header.stamp.sec,
                                     m.header.stamp.nanosec, m.vector.x, m.vector.y])
        print('PASS step/rate/angle limits and graph has no hardware/control publishers')
        print(json.dumps(summary))
    finally:
        peer.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=12)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
    assert process.returncode in (0, 130, -signal.SIGINT)
    executable = Path(get_package_prefix('srm_gimbal_follower'))/'lib/srm_gimbal_follower/gimbal_follower_node'
    for extra in [['-p', 'mode:=hardware'],
                  ['-r', '/srm_aim/gimbal_follower/sim_reference:=/srm_aim/cmd_yaw'],
                  ['-r', '/srm_aim/gimbal_follower/debug:=/srm_aim/cmd_pitch']]:
        result = subprocess.run([str(executable), '--ros-args', *extra], capture_output=True, text=True, timeout=10)
        assert result.returncode == 1 and 'Simulation follower failed:' in result.stderr
    print('PASS hardware mode and hardware-topic remappings rejected')


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir')
    args = parser.parse_args()
    os.environ.setdefault('ROS_DOMAIN_ID', '86')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    main(args.output_dir)
