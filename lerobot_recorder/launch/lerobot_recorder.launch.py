"""
Launch file for lerobot_recorder node.

Usage:
    # Default (uses so101_dual_vr preset)
    ros2 launch lerobot_recorder lerobot_recorder.launch.py

    # With specific preset
    ros2 launch lerobot_recorder lerobot_recorder.launch.py preset:=so101_single_vr

    # With custom config file
    ros2 launch lerobot_recorder lerobot_recorder.launch.py config_file:=/path/to/config.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package share directory
    pkg_share = get_package_share_directory('lerobot_recorder')

    # Launch arguments
    preset_arg = DeclareLaunchArgument(
        'preset',
        default_value='so101_dual_vr',
        description='Preset name (e.g., so101_dual_vr, so101_single_vr, piper_single_vr)'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='',
        description='Custom config file path (overrides preset if provided)'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Determine config file path
    preset = LaunchConfiguration('preset')
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Default preset path
    default_config = PathJoinSubstitution([
        FindPackageShare('lerobot_recorder'),
        'config',
        'presets',
        [preset, '.yaml']
    ])

    # LeRobot recorder node
    lerobot_recorder_node = Node(
        package='lerobot_recorder',
        executable='lerobot_recorder_node',
        name='lerobot_recorder',
        output='screen',
        parameters=[
            default_config,
            {'use_sim_time': use_sim_time}
        ],
    )

    return LaunchDescription([
        preset_arg,
        config_file_arg,
        use_sim_time_arg,
        lerobot_recorder_node,
    ])
