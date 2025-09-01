import os

import glob
from setuptools import setup, find_packages

package_name = 'galum_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aingei',
    maintainer_email='johndoe@example.com',
    description='ROS 2 package for robot control',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cmd_vel_to_motor_speed = galum_robot.cmd_vel_to_motor_speed:main',
            'joystick_node = galum_robot.joystick_node:main',
            'xbox_node = galum_robot.xbox_node:main',
        ],
    },
)
