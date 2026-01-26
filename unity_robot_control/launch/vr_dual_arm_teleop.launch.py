import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, EnvironmentVariable, TextSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # パラメータファイル
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
    enable_bridge_arg = DeclareLaunchArgument(
        'enable_bridge',
        default_value='true',
        description='Enable so101_ros2_bridge (requires lerobot)'
    )

    # lerobot をPythonパスに追加（so101_ros2_bridge が参照）
    lerobot_src = os.path.join(os.path.expanduser('~'), 'ros2_ws', 'src', 'lerobot', 'src')
    set_pythonpath = SetEnvironmentVariable(
        name='PYTHONPATH',
        value=[
            TextSubstitution(text=lerobot_src),
            TextSubstitution(text=':'),
            EnvironmentVariable('PYTHONPATH', default_value=''),
        ],
        condition=IfCondition(LaunchConfiguration('enable_bridge')),
    )
    set_leconda = SetEnvironmentVariable(
        name='LECONDA_SITE_PACKAGES',
        value=TextSubstitution(
            text=os.path.join(
                os.path.expanduser('~'),
                'miniconda3',
                'envs',
                'lerobot',
                'lib',
                'python3.10',
                'site-packages',
            )
        ),
        condition=IfCondition(LaunchConfiguration('enable_bridge')),
    )

    # Unity TCP Endpointノード
    unity_tcp_endpoint = Node(
        package='unity_robot_control',
        executable='unity_tcp_endpoint',
        name='unity_tcp_endpoint',
        parameters=[{
            'ros_ip': LaunchConfiguration('ros_ip'),
            'ros_tcp_port': LaunchConfiguration('ros_tcp_port'),
            'auto_detect_ip': LaunchConfiguration('auto_detect_ip'),
        }],
        output='screen'
    )

    # VR Dual Arm Controlノード（VRがリーダーとして両アームを制御）
    vr_dual_arm_control = Node(
        package='unity_robot_control',
        executable='vr_dual_arm_control_node',
        name='vr_dual_arm_control_node',
        parameters=[config_file],
        output='screen'
    )

    # 左アーム IK Solverノード
    left_arm_ik_solver = Node(
        package='unity_robot_control',
        executable='left_arm_ik_solver_node',
        name='left_arm_ik_solver_node',
        parameters=[config_file],
        output='screen'
    )

    # 右アーム IK Solverノード
    right_arm_ik_solver = Node(
        package='unity_robot_control',
        executable='right_arm_ik_solver_node',
        name='right_arm_ik_solver_node',
        parameters=[config_file],
        output='screen'
    )

    so101_description_pkg = get_package_share_directory('so101_description')
    so101_controller_pkg = get_package_share_directory('so101_controller')
    so101_ros2_bridge_pkg = get_package_share_directory('so101_ros2_bridge')

    model_path = os.path.join(so101_description_pkg, 'urdf', 'so101_new_calib.urdf.xacro')
    controller_config = os.path.join(so101_controller_pkg, 'config', 'so101_vr_controllers.yaml')
    left_bridge_params = os.path.join(so101_ros2_bridge_pkg, 'config', 'so101_left_params.yaml')
    right_bridge_params = os.path.join(so101_ros2_bridge_pkg, 'config', 'so101_right_params.yaml')

    # 左右: robot_state_publisher
    # 左アーム: y=+0.15m（左側）, 右アーム: y=-0.15m（右側）
    left_rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(so101_description_pkg, 'launch', 'rsp.launch.py')
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
            os.path.join(so101_description_pkg, 'launch', 'rsp.launch.py')
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

    # 左右: controller_manager
    left_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        remappings=[('~/robot_description', 'robot_description')],
        output='screen',
        namespace='left_arm',
    )
    right_controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[controller_config],
        remappings=[('~/robot_description', 'robot_description')],
        output='screen',
        namespace='right_arm',
    )

    # 左右: controller spawners
    left_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(so101_controller_pkg, 'launch', 'so101_controllers.launch.py')
        ),
        launch_arguments={'type': 'left_arm'}.items(),
    )
    right_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(so101_controller_pkg, 'launch', 'so101_controllers.launch.py')
        ),
        launch_arguments={'type': 'right_arm'}.items(),
    )

    delayed_left_controllers = TimerAction(period=3.0, actions=[left_controllers])
    delayed_right_controllers = TimerAction(period=3.0, actions=[right_controllers])

    # 左右: SO101 follower bridge (VRでは両腕をフォロワーとして扱う)
    left_bridge = Node(
        package='so101_ros2_bridge',
        executable='follower_ros2_node',
        name='so101_follower_ros2_bridge',
        namespace='left_arm',
        parameters=[left_bridge_params, {'type': 'follower'}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_bridge')),
    )
    right_bridge = Node(
        package='so101_ros2_bridge',
        executable='follower_ros2_node',
        name='so101_follower_ros2_bridge',
        namespace='right_arm',
        parameters=[right_bridge_params, {'type': 'follower'}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_bridge')),
    )

    # IK -> JointTrajectory
    ik_to_traj = Node(
        package='unity_robot_control',
        executable='ik_to_joint_trajectory_node',
        name='ik_to_joint_trajectory_node',
        parameters=[config_file],
        output='screen',
    )

    return LaunchDescription([
        set_pythonpath,
        set_leconda,
        ros_ip_arg,
        ros_tcp_port_arg,
        auto_detect_ip_arg,
        enable_bridge_arg,
        unity_tcp_endpoint,
        vr_dual_arm_control,
        left_arm_ik_solver,
        right_arm_ik_solver,
        ik_to_traj,
        left_rsp,
        right_rsp,
        left_controller_manager,
        right_controller_manager,
        delayed_left_controllers,
        delayed_right_controllers,
        left_bridge,
        right_bridge,
    ])
