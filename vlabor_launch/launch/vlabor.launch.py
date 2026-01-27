import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    vlabor_share = get_package_share_directory('vlabor_launch')
    unity_share = get_package_share_directory('unity_robot_control')
    so101_share = get_package_share_directory('so101_description')

    default_config = os.path.join(vlabor_share, 'config', 'vlabor_profiles.yaml')
    default_rviz = os.path.join(so101_share, 'rviz', 'display.rviz')
    default_rviz_vr = os.path.join(so101_share, 'rviz', 'vr_teleop.rviz')
    default_overhead_cfg = os.path.join(vlabor_share, 'config', 'overhead_camera_c920.yaml')
    default_mirror_single = os.path.join(vlabor_share, 'config', 'copy_map_single.yaml')
    default_mirror_dual = os.path.join(vlabor_share, 'config', 'copy_map_dual.yaml')

    include_profiles = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(unity_share, 'launch', 'teleop_profiles.launch.py')
        ),
        launch_arguments={
            'config': LaunchConfiguration('config'),
            'profile': LaunchConfiguration('profile'),
            'ros_ip': LaunchConfiguration('ros_ip'),
            'ros_tcp_port': LaunchConfiguration('ros_tcp_port'),
            'auto_detect_ip': LaunchConfiguration('auto_detect_ip'),
            'left_serial_port': LaunchConfiguration('left_serial_port'),
            'right_serial_port': LaunchConfiguration('right_serial_port'),
            'left_calibration_path': LaunchConfiguration('left_calibration_path'),
            'right_calibration_path': LaunchConfiguration('right_calibration_path'),
            'driver_backend': LaunchConfiguration('driver_backend'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'gripper_scale': LaunchConfiguration('gripper_scale'),
            'left_webui_port': LaunchConfiguration('left_webui_port'),
            'right_webui_port': LaunchConfiguration('right_webui_port'),
            'enable_foxglove': LaunchConfiguration('enable_foxglove'),
            'enable_bridge': LaunchConfiguration('enable_bridge'),
            'fluent_vision_config': LaunchConfiguration('fluent_vision_config'),
            'fluent_vision_update_serials': LaunchConfiguration('fluent_vision_update_serials'),
            'fluent_vision_serials_path': LaunchConfiguration('fluent_vision_serials_path'),
            'fluent_vision_use_serials_script': LaunchConfiguration('fluent_vision_use_serials_script'),
            'fluent_vision_serials_script_path': LaunchConfiguration('fluent_vision_serials_script_path'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'rviz_vr_config': LaunchConfiguration('rviz_vr_config'),
            'gazebo_args': LaunchConfiguration('gazebo_args'),
            'overhead_camera_config': LaunchConfiguration('overhead_camera_config'),
            'mirror_single_config': LaunchConfiguration('mirror_single_config'),
            'mirror_dual_config': LaunchConfiguration('mirror_dual_config'),
            'dashboard_port': LaunchConfiguration('dashboard_port'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'config',
            default_value=default_config,
            description='Path to vlabor launch profiles YAML',
        ),
        DeclareLaunchArgument(
            'profile',
            default_value='so101_vr_dual_teleop',
            description='Profile name from vlabor_profiles.yaml',
        ),
        DeclareLaunchArgument('ros_ip', default_value=''),
        DeclareLaunchArgument('ros_tcp_port', default_value=''),
        DeclareLaunchArgument('auto_detect_ip', default_value=''),
        DeclareLaunchArgument('left_serial_port', default_value=''),
        DeclareLaunchArgument('right_serial_port', default_value=''),
        DeclareLaunchArgument('left_calibration_path', default_value=''),
        DeclareLaunchArgument('right_calibration_path', default_value=''),
        DeclareLaunchArgument('driver_backend', default_value=''),
        DeclareLaunchArgument('auto_enable', default_value=''),
        DeclareLaunchArgument('gripper_scale', default_value=''),
        DeclareLaunchArgument('left_webui_port', default_value=''),
        DeclareLaunchArgument('right_webui_port', default_value=''),
        DeclareLaunchArgument('enable_foxglove', default_value=''),
        DeclareLaunchArgument('enable_bridge', default_value=''),
        DeclareLaunchArgument('fluent_vision_config', default_value=''),
        DeclareLaunchArgument('fluent_vision_update_serials', default_value=''),
        DeclareLaunchArgument('fluent_vision_serials_path', default_value=''),
        DeclareLaunchArgument('fluent_vision_use_serials_script', default_value=''),
        DeclareLaunchArgument('fluent_vision_serials_script_path', default_value=''),
        DeclareLaunchArgument('rviz_config', default_value=''),
        DeclareLaunchArgument('rviz_vr_config', default_value=''),
        DeclareLaunchArgument('gazebo_args', default_value=''),
        DeclareLaunchArgument('overhead_camera_config', default_value=''),
        DeclareLaunchArgument('mirror_single_config', default_value=''),
        DeclareLaunchArgument('mirror_dual_config', default_value=''),
        DeclareLaunchArgument('dashboard_port', default_value=''),
        include_profiles,
    ])
