# MIT License
#
# Copyright (c) 2025 nimiCurtis
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch configurations
    model = LaunchConfiguration('model')
    mode = LaunchConfiguration('mode')
    robot_type = LaunchConfiguration('type')
    # TF offset from world to base_link (x, y, z)
    tf_offset_x = LaunchConfiguration('tf_offset_x')
    tf_offset_y = LaunchConfiguration('tf_offset_y')
    tf_offset_z = LaunchConfiguration('tf_offset_z')

    # Declare launch arguments with default values for backward compatibility
    tf_offset_x_arg = DeclareLaunchArgument(
        'tf_offset_x', default_value='0.0',
        description='TF offset X from world to base_link'
    )
    tf_offset_y_arg = DeclareLaunchArgument(
        'tf_offset_y', default_value='0.0',
        description='TF offset Y from world to base_link'
    )
    tf_offset_z_arg = DeclareLaunchArgument(
        'tf_offset_z', default_value='0.0',
        description='TF offset Z from world to base_link'
    )

    # Determine use_sim based on mode
    use_sim = PythonExpression(["'", mode, "' != 'real'"])

    # Robot description
    robot_description = ParameterValue(
        Command(
            [
                PathJoinSubstitution([FindExecutable(name='xacro')]),
                ' ',
                model,
                ' ',
                'mode:=',
                mode,
            ]
        ),
        value_type=str,
    )

    # Robot state publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'robot_description': robot_description},
            {'use_sim_time': use_sim},
            {
                'frame_prefix': [
                    robot_type,
                    TextSubstitution(text='/'),
                ]
            },
        ],
        namespace=robot_type,
    )

    # Static transform publisher: world -> {robot_type}/base_link
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='world_to_base_link_publisher',
        namespace=robot_type,
        arguments=[
            tf_offset_x,
            tf_offset_y,
            tf_offset_z,  # x y z
            '0',
            '0',
            '0',  # roll pitch yaw (or qx qy qz qw)
            'world',  # parent frame
            [robot_type, TextSubstitution(text='/base_link')],  # child frame
        ],
    )

    return LaunchDescription(
        [
            tf_offset_x_arg,
            tf_offset_y_arg,
            tf_offset_z_arg,
            robot_state_publisher_node,
            static_tf_node,
        ]
    )
