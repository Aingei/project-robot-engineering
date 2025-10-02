# camera_web.launch.py
from pathlib import Path
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    home = str(Path.home())
    hls_dir = "/dev/shm/hls" 
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
        # --- in your launch file ---
        ExecuteProcess(
            cmd=[
                'ffmpeg',
                '-hide_banner','-loglevel','warning',

                # v4l2 raw capture (lower latency than MJPEG)
                '-f','v4l2',
                '-input_format','yuyv422',
                '-framerate','20',
                '-video_size','640x480',            # <-- use a real YUYV mode
                '-thread_queue_size','512',
                '-i','/dev/video0',

                # scale to 640x360 for HLS + set output pixfmt
                '-vf','scale=640:360,format=yuv420p',

                # encoder + rate control (fix your warning: set maxrate >= b:v)
                '-c:v','libx264','-preset','veryfast','-tune','zerolatency',
                '-b:v','700k','-maxrate','700k','-bufsize','350k',

                # 20 fps => keyframe every 0.5s (matches hls_time=0.5)
                '-g','10','-keyint_min','10','-sc_threshold','0',
                '-an',

                # (optional) shave input buffering further
                '-fflags','nobuffer','-flags','low_delay','-use_wallclock_as_timestamps','1',

                # HLS: tiny window for low latency
                '-f','hls',
                '-hls_time','0.5',
                '-hls_list_size','2',
                '-hls_delete_threshold','2',
                '-hls_flags','delete_segments+independent_segments+omit_endlist',
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