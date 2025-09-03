import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Define the path to the launch file directory
    launch_file_dir = os.path.join(get_package_share_directory('galum_robot'), 'launch')
    
    # Include microros.launch.py
    microros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'microros.launch.py')
        )
    )
    
    galum_speed= Node(
        package="galum_robot",
        executable="galum_speed",
        name="Cmd_Vel_To_Rpm",
        # output="screen",
        namespace="",
        # parameters=[motor_config], #Testing
    )

    # Add actions to the launch description
    ld.add_action(microros_launch)
    ld.add_action(galum_speed)

    return ld

if __name__ == '__main__':
    generate_launch_description()