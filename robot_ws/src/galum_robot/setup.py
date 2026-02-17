import os

import glob
from setuptools import setup, find_packages

package_name = 'galum_robot'

# ----- add this helper before setup() -----
def all_files_in_dir(root_dir):
    """Recursively collect all files under a directory."""
    file_list = []
    for dirpath, _, filenames in os.walk(root_dir):
        for f in filenames:
            file_list.append(os.path.join(dirpath, f))
    return file_list

# Replace only this part dynamically before calling setup()
www_files = all_files_in_dir('www') if os.path.isdir('www') else []

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'www'), www_files),

    ],
    
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='aingei',
    maintainer_email='johndoe@example.com',
    description='ROS 2 package for robot control',
    license='MIT',
    # tests_require=['pytest'],
    extras_require={
        # optional: install with `pip install .[test]` if you ever need it
        'test': ['pytest']
    },
    entry_points={
        'console_scripts': [
            'galum_speed = galum_robot.galum_speed:main',
            'showcamera = galum_robot.showcamera:main',
            'joystick_node = galum_robot.joystick_node:main',
            'xbox_node = galum_robot.xbox_node:main',
            'stepper= galum_robot.stepper:main',
            'servo= galum_robot.servo:main',
            'autowalk = galum_robot.autowalk:main',
        ],
    },
)
