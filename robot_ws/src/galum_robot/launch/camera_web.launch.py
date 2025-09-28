# camera_web.launch.py
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    home = str(Path.home())
    hls_dir = f"{home}/hls"
    hls_out = f"{hls_dir}/stream.m3u8"

    return LaunchDescription([
        # 0) Prepare output dir (and clear old HLS files)
        ExecuteProcess(
            cmd=['bash', '-lc', f'mkdir -p "{hls_dir}" && rm -f "{hls_dir}"/stream_*.ts "{hls_out}"'],
            output='screen'
        ),

        # 1) rosbridge websocket
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_websocket',
            output='screen',
            parameters=[{'port': 9090}],
        ),

        # 2) ffmpeg: USB camera -> low-latency, compressed HLS (480p, ~0.8–0.9 Mbps)
        ExecuteProcess(
            cmd=[
                'ffmpeg',
                '-hide_banner', '-loglevel', 'warning',
                '-f', 'v4l2',
                '-input_format', 'mjpeg',
                '-framerate', '20',
                '-video_size', '640x360',        # <-- fixed the missing quote
                '-thread_queue_size', '512',
                '-i', '/dev/video0',

                # Colors / pixel format + downscale (keep even width)
                '-vf', 'scale=640:360,format=yuv420p',

                # Encoder settings (small + low latency)
                '-c:v', 'libx264', '-preset', 'veryfast', '-tune', 'zerolatency',
                '-b:v', '800k', '-maxrate', '500k', '-bufsize', '1000k',

                # 20 fps -> keyframe every 0.5 s (aligns with hls_time=0.5)
                '-g', '10', '-keyint_min', '10', '-sc_threshold', '0',
                '-an',

                # HLS output (tiny window, low latency)
                '-f', 'hls',
                '-hls_time', '0.5',
                '-hls_list_size', '2',
                '-hls_delete_threshold', '2',
                '-hls_flags', 'delete_segments+independent_segments+omit_endlist',
                hls_out
            ],
            output='screen'
        ),

        # 3) Static web server for the HLS directory
        ExecuteProcess(
            cmd=['python3', '-m', 'http.server', '8000', '--directory', hls_dir],
            output='screen'
        ),
    ])
