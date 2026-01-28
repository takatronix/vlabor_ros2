from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'vlabor_launch'

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
        (os.path.join('share', package_name, 'config', 'profiles'), glob('config/profiles/*.yaml')),
        (os.path.join('share', package_name, 'config', 'device_assignments'),
            glob('config/device_assignments/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takashi Otsuka',
    maintainer_email='takatronix@gmail.com',
    description='Launch entrypoints and profiles for vlabor teleop',
    license='Apache-2.0',
    tests_require=['pytest'],
)
