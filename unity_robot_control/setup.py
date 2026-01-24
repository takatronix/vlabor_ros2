from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'unity_robot_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
        (os.path.join('share', package_name, 'config', 'positions'), glob('config/positions/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takashi Otsuka',
    maintainer_email='takatronix@gmail.com',
    description='Unity teleoperation and robot control package',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unity_tcp_endpoint = unity_robot_control.unity_tcp_endpoint:main',
            'so101_control_node = unity_robot_control.so101_control_node:main',
            'ik_solver_node = unity_robot_control.ik_solver_node:main',
            'vr_dual_arm_control_node = unity_robot_control.vr_dual_arm_control_node:main',
            'ik_to_joint_trajectory_node = unity_robot_control.ik_to_joint_trajectory_node:main',
            'joint_state_mirror_node = unity_robot_control.joint_state_mirror_node:main',
            # 左アーム/右アーム用IKソルバー
            'left_arm_ik_solver_node = unity_robot_control.ik_solver_node:main_left_arm',
            'right_arm_ik_solver_node = unity_robot_control.ik_solver_node:main_right_arm',
            # LeRobot変換ツール
            'rosbag_to_lerobot = unity_robot_control.rosbag_to_lerobot:main',
            # WebUI
            'so101_webui_node = unity_robot_control.webui.webui_node:main',
        ],
    },
)
