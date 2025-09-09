import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()

    motor_config = os.path.join(
        get_package_share_directory('galum_robot'),
        'config',
        'motor_config.yaml'
    )
    
    stepper_simple = Node(
        package="galum_robot",
        executable="stepper_simple",
        name="stepper_node",
        # output="screen",
        namespace="",
    )
    
    galum_speed = Node(
        package="galum_robot",
        executable="galum_speed",
        name="Cmd_Vel_To_Rpm",
        # output="screen",
        namespace="",
        # parameters=[motor_config], #Testing
    )
    
    node_microros_1 = Node(
        package="micro_ros_agent",
        executable="micro_ros_agent",
        output="screen",
        arguments=["serial", "--dev", "/dev/ttyUSB0"],
    )
    
    ld.add_action(node_microros_1)
    ld.add_action(galum_speed)
    ld.add_action(stepper_simple)

    return ld