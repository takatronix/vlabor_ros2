from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'vlabor_dashboard'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Takashi Otsuka',
    maintainer_email='takatronix@gmail.com',
    description='Unified dashboard for vlabor teleop profiles',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vlabor_dashboard_node = vlabor_dashboard.dashboard_node:main',
        ],
    },
)
