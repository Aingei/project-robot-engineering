# camera_web_webrtc.launch.py
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    ffmpeg_cmd = (
        'ffmpeg -f v4l2 -framerate 30 -video_size 1280x720 -i /dev/video0 '
        '-use_wallclock_as_timestamps 1 -fflags nobuffer -flags low_delay '
        '-vf format=yuv420p '
        '-c:v h264_v4l2m2m -b:v 2000k -tune zerolatency -g 30 -bf 0 -an '
        ' -f rtsp rtsp://172.20.10.3:8554/cam'
    )
    #172.20.10.3
    return LaunchDescription([
        # rosbridge (if you still need JS <-> ROS)
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090}],
        ),

        # ffmpeg publisher -> MediaMTX on the Pi
        ExecuteProcess(cmd=['bash', '-lc', ffmpeg_cmd], output='screen'),
    ])
