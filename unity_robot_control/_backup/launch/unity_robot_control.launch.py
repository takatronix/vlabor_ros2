from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # パラメータ
    ros_ip_arg = DeclareLaunchArgument(
        'ros_ip',
        default_value='0.0.0.0',
        description='ROS TCP Endpoint IP address'
    )
    
    ros_tcp_port_arg = DeclareLaunchArgument(
        'ros_tcp_port',
        default_value='10000',
        description='ROS TCP Endpoint port'
    )
    
    auto_detect_ip_arg = DeclareLaunchArgument(
        'auto_detect_ip',
        default_value='true',
        description='Auto-detect local IP address'
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
    
    # SO101制御ノード
    so101_control = Node(
        package='unity_robot_control',
        executable='so101_control_node',
        name='so101_control_node',
        output='screen'
    )
    
    # IK計算ノード
    ik_solver = Node(
        package='unity_robot_control',
        executable='ik_solver_node',
        name='ik_solver_node',
        output='screen'
    )
    
    return LaunchDescription([
        ros_ip_arg,
        ros_tcp_port_arg,
        auto_detect_ip_arg,
        unity_tcp_endpoint,
        so101_control,
        ik_solver,
    ])
