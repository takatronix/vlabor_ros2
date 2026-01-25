from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'lerobot_runner'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config', 'presets'), glob('config/presets/*.yaml')),
        (os.path.join('share', package_name, 'config', 'robots'), glob('config/robots/*.yaml')),
        (os.path.join('share', package_name, 'config', 'policies'), glob('config/policies/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'aiohttp',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='ros2',
    maintainer_email='ros2@example.com',
    description='LeRobot policy runner for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'policy_runner_node = lerobot_runner.policy_runner_node:main',
        ],
    },
)
