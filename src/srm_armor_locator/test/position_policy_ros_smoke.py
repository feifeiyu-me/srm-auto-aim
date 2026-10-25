"""Opt-in policy only in isolated synthetic ROS tests, never hardware activation."""
from copy import deepcopy
import os
from pathlib import Path
import sys
from ippe_ros_smoke import run


def legacy(frame):
    result = deepcopy(frame)
    for location in result.locations:
        location.position_valid = location.orientation_valid = False
        location.position.x = location.position.y = location.position.z = 0.0
        location.position_reason = ''
    return result


def main(calibration):
    baseline = run(calibration, False)
    retained = run(calibration, True, [
        '-p', 'allow_rotation_ambiguous_position:=true',
        '-p', 'position_only_max_translation_m:=0.002'])
    limited = run(calibration, False, [
        '-p', 'allow_rotation_ambiguous_position:=true',
        '-p', 'position_only_max_translation_m:=0.000000000001'])
    translation = run(calibration, False, [
        '-p', 'allow_rotation_ambiguous_position:=true',
        '-p', 'position_only_max_translation_m:=1.0',
        '-p', 'ambiguity_translation_m:=0.000000000001'])
    for seq, frame in baseline.items():
        rejected, accepted = frame.locations
        assert not rejected.position_valid and not rejected.orientation_valid
        assert accepted.position_valid and accepted.orientation_valid and accepted.pose_valid
        assert accepted.position == accepted.pose.position
        item = retained[seq].locations[0]
        assert item.position_valid and not item.orientation_valid
        assert not item.pose_valid and item.ambiguous and item.reason == 'ambiguous_ippe'
        assert item.position_reason == 'position_retained_rotation_ambiguity'
        assert abs(item.position.z-4.7) < .01
        assert item.pose.position.z == item.axial_depth_m == item.distance_m == 0.0
        assert not item.digit_verified and not item.digit_check_enabled and item.digit == 0
        assert legacy(frame) == legacy(retained[seq]), 'legacy location fields changed'
        assert not limited[seq].locations[0].position_valid
        assert limited[seq].locations[0].position_reason == 'position_translation_limit_exceeded'
        assert not translation[seq].locations[0].position_valid
        assert translation[seq].locations[0].position_reason == 'translation_ambiguity_rejected'
    print('PASS default strict policy, independent position/orientation flags and unchanged legacy fields')
    print('PASS position limit rejects; original translation ambiguity rejects despite permissive position limit')


if __name__ == '__main__':
    os.environ.setdefault('ROS_DOMAIN_ID', '83')
    os.environ['ROS_LOCALHOST_ONLY'] = '1'
    main(str(Path(sys.argv[1]).resolve()))
