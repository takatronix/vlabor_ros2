#!/usr/bin/env python3
"""
SO101 WebUI Launch File
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'web_port',
            default_value='8080',
            description='WebUI server port'
        ),
        DeclareLaunchArgument(
            'arm_namespaces',
            default_value='[left_arm, right_arm]',
            description='List of arm namespaces to monitor'
        ),

        Node(
            package='unity_robot_control',
            executable='so101_webui_node',
            name='so101_webui_node',
            parameters=[{
                'web_port': LaunchConfiguration('web_port'),
            }],
            output='screen',
        ),
    ])
