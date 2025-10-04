# web_cam_dashboard.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import glob, os

def launch_setup(context, *args, **kwargs):
    rosbridge_port = int(LaunchConfiguration('rosbridge_port').perform(context))
    web_video_port = int(LaunchConfiguration('web_video_port').perform(context))
    width  = int(LaunchConfiguration('width').perform(context))
    height = int(LaunchConfiguration('height').perform(context))
    fps    = int(LaunchConfiguration('fps').perform(context))
    max_cams = int(LaunchConfiguration('max_cams').perform(context))

    # AUTO-DISCOVER: all current /dev/video*
    all_devices = sorted(glob.glob('/dev/video*'))
    devices = all_devices[:max_cams] if max_cams > 0 else all_devices

    nodes = []

    if not devices:
        nodes.append(LogInfo(msg='[web_cam_dashboard] No /dev/video* devices found.'))
    else:
        nodes.append(LogInfo(msg=f'[web_cam_dashboard] Found devices: {", ".join(devices)}'))

    # 1) One v4l2 node per device -> /cameras/<videoXX>/image_raw
    for dev in devices:
        base = os.path.basename(dev)  # e.g. "video25"
        topic = f'/cameras/{base}/image_raw'
        nodes.append(Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name=f'usb_cam_{base}',
            output='screen',
            parameters=[{
                'video_device': dev,
                'image_size': [width, height],
                'time_per_frame': [1, fps],
                'camera_frame_id': f'camera_frame_{base}',
            }],
            remappings=[('image_raw', topic)]
        ))

    # 2) rosbridge
    nodes.append(Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': rosbridge_port, 'address': '0.0.0.0'}],
    ))

    # 2.5) rosapi (so the web page can list topics)
    nodes.append(Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen',
    ))

    # 3) web_video_server
    nodes.append(Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{'port': web_video_port, 'address': '0.0.0.0'}],
    ))

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        DeclareLaunchArgument('web_video_port', default_value='8080'),
        DeclareLaunchArgument('width',  default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps',    default_value='30'),
        # 0 = unlimited; otherwise cap number of cameras to spawn
        DeclareLaunchArgument('max_cams', default_value='8'),
        OpaqueFunction(function=launch_setup),
    ])
