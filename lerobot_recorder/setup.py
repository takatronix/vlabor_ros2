from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'lerobot_recorder'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        # Config files - robots
        (os.path.join('share', package_name, 'config', 'robots'),
            glob('config/robots/*.yaml')),
        # Config files - presets
        (os.path.join('share', package_name, 'config', 'presets'),
            glob('config/presets/*.yaml')),
    ],
    install_requires=[
        'setuptools',
        'aiohttp',
        'numpy',
        'pandas',
        'pyarrow',
    ],
    zip_safe=True,
    maintainer='takatronix',
    maintainer_email='takatronix@example.com',
    description='LeRobot format episode recorder for ROS2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'episode_recorder_node = lerobot_recorder.episode_recorder_node:main',
        ],
    },
)
