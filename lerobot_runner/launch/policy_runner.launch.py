"""Launch file for policy runner node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    preset_arg = DeclareLaunchArgument(
        'preset',
        default_value='so101_dual_smolvla',
        description='Preset configuration name'
    )

    # Get preset config path
    preset = LaunchConfiguration('preset')
    config_file = PathJoinSubstitution([
        FindPackageShare('lerobot_runner'),
        'config', 'presets',
        [preset, '.yaml']
    ])

    # Create node
    policy_runner_node = Node(
        package='lerobot_runner',
        executable='policy_runner_node',
        name='policy_runner',
        output='screen',
        parameters=[config_file],
    )

    return LaunchDescription([
        preset_arg,
        policy_runner_node,
    ])
