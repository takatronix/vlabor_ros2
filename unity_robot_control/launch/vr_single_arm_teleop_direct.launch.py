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

    ros_ip_arg = DeclareLaunchArgument(
        'ros_ip',
        default_value='0.0.0.0',
        description='ROS TCP Endpoint IP address'
    )
    ros_tcp_port_arg = DeclareLaunchArgument(
        'ros_tcp_port',
        default_value='42000',
        description='ROS TCP Endpoint port'
    )
    auto_detect_ip_arg = DeclareLaunchArgument(
        'auto_detect_ip',
        default_value='true',
        description='Auto-detect local IP address'
    )
    serial_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Arm serial port'
    )
    so101_desc_pkg = get_package_share_directory('so101_description')
    default_calib_path = os.path.join(so101_desc_pkg, 'config', 'calibration', 'left_arm.json')
    calib_arg = DeclareLaunchArgument(
        'calibration_path',
        default_value=default_calib_path,
        description='Arm calibration JSON (ROS2 format)'
    )
    driver_backend_arg = DeclareLaunchArgument(
        'driver_backend',
        default_value='mock',
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
        description='Gripper open angle scale (1.0 = normal)'
    )

    unity_tcp_endpoint = Node(
        package='ros_tcp_endpoint',
        executable='default_server_endpoint',
        name='unity_tcp_endpoint',
        parameters=[{
            'ROS_IP': LaunchConfiguration('ros_ip'),
            'ROS_TCP_PORT': LaunchConfiguration('ros_tcp_port'),
        }],
        output='screen'
    )

    vr_dual_arm_control = Node(
        package='unity_robot_control',
        executable='vr_dual_arm_control_node',
        name='vr_dual_arm_control_node',
        parameters=[config_file],
        output='screen'
    )

    left_arm_ik_solver = Node(
        package='unity_robot_control',
        executable='left_arm_ik_solver_node',
        name='left_arm_ik_solver_node',
        parameters=[config_file],
        output='screen'
    )

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

    left_so101_control = Node(
        package='unity_robot_control',
        executable='so101_control_node',
        name='so101_control_node',
        namespace='left_arm',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'calibration_path': LaunchConfiguration('calibration_path'),
            'driver_backend': LaunchConfiguration('driver_backend'),
            'auto_enable': LaunchConfiguration('auto_enable'),
            'gripper_scale': LaunchConfiguration('gripper_scale'),
            'urdf_path': model_path,
        }],
        remappings=[('joint_ctrl_single', 'ik/joint_angles')],
        output='screen',
    )

    return LaunchDescription([
        ros_ip_arg,
        ros_tcp_port_arg,
        auto_detect_ip_arg,
        serial_arg,
        calib_arg,
        driver_backend_arg,
        auto_enable_arg,
        gripper_scale_arg,
        unity_tcp_endpoint,
        vr_dual_arm_control,
        left_arm_ik_solver,
        left_rsp,
        left_so101_control,
    ])
