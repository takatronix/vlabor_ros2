"""
VR Dual Arm Teleop - Robot Only (リモート用)

Unity TCP Endpointなしでロボット制御ノードのみ起動する。
VR側からはZenoh Bridge経由でトピックが届く。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config_file = PathJoinSubstitution([
        FindPackageShare('unity_robot_control'),
        'config',
        'vr_dual_arm_config.yaml'
    ])

    # Arguments
    left_serial_arg = DeclareLaunchArgument(
        'left_serial_port',
        default_value='/dev/ttyACM0',
        description='Left arm serial port'
    )
    right_serial_arg = DeclareLaunchArgument(
        'right_serial_port',
        default_value='/dev/ttyACM1',
        description='Right arm serial port'
    )

    so101_desc_pkg = get_package_share_directory('so101_description')
    default_left_calib_path = os.path.join(so101_desc_pkg, 'config', 'calibration', 'left_arm.json')
    default_right_calib_path = os.path.join(so101_desc_pkg, 'config', 'calibration', 'right_arm.json')

    left_calib_arg = DeclareLaunchArgument(
        'left_calibration_path',
        default_value=default_left_calib_path,
        description='Left arm calibration JSON'
    )
    right_calib_arg = DeclareLaunchArgument(
        'right_calibration_path',
        default_value=default_right_calib_path,
        description='Right arm calibration JSON'
    )
    driver_backend_arg = DeclareLaunchArgument(
        'driver_backend',
        default_value='feetech',
        description='SO101 driver backend (mock | feetech)'
    )
    auto_enable_arg = DeclareLaunchArgument(
        'auto_enable',
        default_value='false',
        description='Auto enable on startup'
    )
    gripper_scale_arg = DeclareLaunchArgument(
        'gripper_scale',
        default_value='2.0',
        description='Gripper open angle scale'
    )

    # VR Control Node (VR入力をロボット出力に変換)
    vr_dual_arm_control = Node(
        package='unity_robot_control',
        executable='vr_dual_arm_control_node',
        name='vr_dual_arm_control_node',
        parameters=[config_file],
        output='screen'
    )

    # IK Solvers
    left_arm_ik_solver = Node(
        package='unity_robot_control',
        executable='left_arm_ik_solver_node',
        name='left_arm_ik_solver_node',
        parameters=[config_file],
        output='screen'
    )
    right_arm_ik_solver = Node(
        package='unity_robot_control',
        executable='right_arm_ik_solver_node',
        name='right_arm_ik_solver_node',
        parameters=[config_file],
        output='screen'
    )

    # Robot State Publishers (URDF/TF)
    model_path = os.path.join(so101_desc_pkg, 'urdf', 'so101_new_calib.urdf.xacro')

    left_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(so101_desc_pkg, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'model': model_path,
            'mode': 'real',
            'type': 'left_arm',
            'tf_offset_x': '0.0',
            'tf_offset_y': '0.15',
            'tf_offset_z': '0.0',
        }.items(),
    )
    right_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(so101_desc_pkg, 'launch', 'rsp.launch.py')
        ),
        launch_arguments={
            'model': model_path,
            'mode': 'real',
            'type': 'right_arm',
            'tf_offset_x': '0.0',
            'tf_offset_y': '-0.15',
            'tf_offset_z': '0.0',
        }.items(),
    )

    # SO101 Control Nodes (実機制御)
    left_so101_control = Node(
        package='unity_robot_control',
        executable='so101_control_node',
        name='so101_control_node',
        namespace='left_arm',
        parameters=[{
            'serial_port': LaunchConfiguration('left_serial_port'),
            'calibration_path': LaunchConfiguration('left_calibration_path'),
            'driver_backend': LaunchConfiguration('driver_backend'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'gripper_scale': LaunchConfiguration('gripper_scale'),
            'urdf_path': model_path,
        }],
        remappings=[('joint_ctrl_single', 'ik/joint_angles')],
        output='screen',
    )
    right_so101_control = Node(
        package='unity_robot_control',
        executable='so101_control_node',
        name='so101_control_node',
        namespace='right_arm',
        parameters=[{
            'serial_port': LaunchConfiguration('right_serial_port'),
            'calibration_path': LaunchConfiguration('right_calibration_path'),
            'driver_backend': LaunchConfiguration('driver_backend'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'gripper_scale': LaunchConfiguration('gripper_scale'),
            'urdf_path': model_path,
        }],
        remappings=[('joint_ctrl_single', 'ik/joint_angles')],
        output='screen',
    )

    return LaunchDescription([
        left_serial_arg,
        right_serial_arg,
        left_calib_arg,
        right_calib_arg,
        driver_backend_arg,
        auto_enable_arg,
        gripper_scale_arg,
        vr_dual_arm_control,
        left_arm_ik_solver,
        right_arm_ik_solver,
        left_rsp,
        right_rsp,
        left_so101_control,
        right_so101_control,
    ])
