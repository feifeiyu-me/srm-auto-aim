"""Only two synthetic nodes; no camera, old control, serial or hardware nodes."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('srm_gimbal_follower'))/'config/simulation.yaml')
    return LaunchDescription([
        Node(package='srm_gimbal_follower', executable='gimbal_follower_node',
             parameters=[config], output='screen', remappings=[
                 ('/srm_aim/visual_follower/debug', '/srm_aim/gimbal_follower/sim_visual_debug'),
                 ('/srm_aim/gimbal_state_stamped', '/srm_aim/gimbal_follower/sim_state_stamped')]),
        Node(package='srm_gimbal_follower', executable='gimbal_simulator_node',
             parameters=[config], output='screen'),
    ])
