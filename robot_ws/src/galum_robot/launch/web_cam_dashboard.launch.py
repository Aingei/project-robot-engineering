# web_cam_dashboard.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):
    # Resolve args to concrete values
    rosbridge_port = LaunchConfiguration('rosbridge_port').perform(context)
    web_video_port = LaunchConfiguration('web_video_port').perform(context)
    width  = int(LaunchConfiguration('width').perform(context))
    height = int(LaunchConfiguration('height').perform(context))
    fps    = int(LaunchConfiguration('fps').perform(context))

    nodes = []

    # 1) Camera (USB via v4l2_camera) -> publishes /camera/image_raw
    nodes.append(Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='usb_cam',
        output='screen',
        parameters=[{
            'image_size': [width, height],     # list[int, int]
            'time_per_frame': [1, fps],        # [num, den] -> 1/fps seconds
            'camera_frame_id': 'camera_frame',
        }],
        remappings=[('image_raw', '/camera/image_raw')]
    ))

    # 2) rosbridge
    nodes.append(Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{
            'port': int(rosbridge_port),
            'address': '0.0.0.0',
        }],
    ))

    # 3) web_video_server (MJPEG)
    nodes.append(Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{
            'port': int(web_video_port),
            'address': '0.0.0.0',
        }],
    ))

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('web_video_port', default_value='8080'),
        DeclareLaunchArgument('width',  default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps',    default_value='30'),
        OpaqueFunction(function=launch_setup),
    ])
