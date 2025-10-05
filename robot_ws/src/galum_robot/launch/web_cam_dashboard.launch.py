# web_cam_dashboard.launch.py
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import glob, os, re, hashlib

# ---------- Helpers ----------
def is_usb_video(dev_path: str) -> bool:
    base = os.path.basename(dev_path)
    sysdev = f'/sys/class/video4linux/{base}/device'
    try:
        real = os.path.realpath(sysdev).lower()
        return ('/usb' in real) or ('xhci' in real) or ('usb' in real)
    except Exception:
        return False

def list_capture_nodes_by_id():
    # เอาเฉพาะโหนด capture จริง (index0)
    return sorted(glob.glob('/dev/v4l/by-id/*-video-index0'))

ALIAS_PATTERNS = [
    (re.compile(r'c922', re.I),  'cam_front'),
    (re.compile(r'jieli', re.I), 'cam_back'),
    # เพิ่มแพทเทิร์นของคุณได้ตรงนี้ เช่น (re.compile(r'c925e', re.I), 'cam_side')
]

def alias_from_byid(path: str, used_aliases: set) -> str:
    base = os.path.basename(path)  # ex: usb-046d_C922_Pro_Stream_Webcam_98DDB7AF-video-index0
    for patt, alias in ALIAS_PATTERNS:
        if patt.search(base) and alias not in used_aliases:
            return alias
    # ถ้าไม่แมช ให้สร้าง alias คงที่จาก hash ของชื่อ
    h = hashlib.sha1(base.encode()).hexdigest()[:6]
    alias = f'cam_{h}'
    i = 1
    orig = alias
    while alias in used_aliases:
        alias = f'{orig}_{i}'; i += 1
    return alias

def discover_and_name(max_cams: int):
    pairs = []  # [(device_path, alias)]
    used = set()

    byid = list_capture_nodes_by_id()
    if max_cams > 0:
        byid = byid[:max_cams]
    for d in byid:
        a = alias_from_byid(d, used)
        pairs.append((d, a))
        used.add(a)

    # fallback ถ้าเครื่องไม่มี by-id (ไม่น่าจะเกิดใน Pi5)
    if not pairs:
        usb_videos = [d for d in sorted(glob.glob('/dev/video[0-9]*')) if is_usb_video(d)]
        if max_cams > 0:
            usb_videos = usb_videos[:max_cams]
        for d in usb_videos:
            base = os.path.basename(d)
            a = f'cam_{base}'
            i = 1
            while a in used:
                a = f'cam_{base}_{i}'; i += 1
            pairs.append((d, a)); used.add(a)
    return pairs

# ---------- Launch setup ----------
def launch_setup(context, *args, **kwargs):
    rosbridge_port = int(LaunchConfiguration('rosbridge_port').perform(context))
    web_video_port = int(LaunchConfiguration('web_video_port').perform(context))
    width  = int(LaunchConfiguration('width').perform(context))
    height = int(LaunchConfiguration('height').perform(context))
    fps    = int(LaunchConfiguration('fps').perform(context))
    max_cams = int(LaunchConfiguration('max_cams').perform(context))

    nodes = []

    pairs = discover_and_name(max_cams)  # [(device_path, alias)]
    if not pairs:
        nodes.append(LogInfo(msg='[web_cam_dashboard] No camera devices found (by-id/usb).'))
        return nodes

    nodes.append(LogInfo(msg='[web_cam_dashboard] Using: ' + ', '.join([f'{a}({d})' for d,a in pairs])))

    # ดีฟอลต์ (ปลอดภัย + เสถียร)
    DEFAULT_PARAMS = {
        'image_size': [width, height],
        'time_per_frame': [1, fps],
        'io_method': 'mmap',
        'pixel_format': 'YUYV',        # ← บังคับ YUYV เป็น default
        'output_encoding': 'rgb8',
    }

    # จูนรายกล้องตาม alias ที่เราตั้ง
    OVERRIDES = {
        # Logitech C922: MJPG ที่เสถียร
        'cam_front': {
            'pixel_format': 'MJPEG',     # ← MJPG ใช้ได้กับ C922 เท่านั้น
            'image_size': [1280, 720],
            'time_per_frame': [1, 30],
            
        },
        # Jieli: ใช้ YUYV เท่านั้น
        'cam_back': {
            'pixel_format': 'YUYV',
            'image_size': [320, 240],
            'time_per_frame': [1, 30],
            'output_encoding': 'bgr8',
            'io_method': 'mmap',
            'use_sensor_data_qos': True,
        }


    }


    # Spawn ทุกกล้อง
    for dev, alias in pairs:
        params = dict(DEFAULT_PARAMS)
        params.update({'video_device': dev, 'camera_frame_id': f'frame_{alias}'})
        if alias in OVERRIDES:
            params.update(OVERRIDES[alias])
            # params.setdefault('output_encoding', 'rgb8')

        topic = f'/cameras/{alias}/image_raw'
        nodes.append(Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name=f'usb_cam_{alias}',
            output='screen',
            parameters=[params],
            remappings=[('image_raw', topic)]
        ))

    # rosbridge
    nodes.append(Node(
        package='rosbridge_server',
        executable='rosbridge_websocket',
        name='rosbridge_websocket',
        output='screen',
        parameters=[{'port': rosbridge_port, 'address': '0.0.0.0'}],
    ))

    # rosapi (ให้หน้าเว็บ list topics ได้)
    nodes.append(Node(
        package='rosapi',
        executable='rosapi_node',
        name='rosapi',
        output='screen',
    ))

    # web_video_server (MJPEG streamer)
    nodes.append(Node(
        package='web_video_server',
        executable='web_video_server',
        name='web_video_server',
        output='screen',
        parameters=[{
            'port': web_video_port,
            'address': '0.0.0.0',
            'acceptable_qos_profiles': ['sensor_data', 'default'],
        }],
    ))
    

    return nodes

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('rosbridge_port', default_value='9090'),
        # DeclareLaunchArgument('web_video_port', default_value='5500'),  # ค่าเริ่ม 5500 ตามที่ใช้หน้าเว็บ
        DeclareLaunchArgument('web_video_port', default_value='8080'),
        DeclareLaunchArgument('width',  default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps',    default_value='30'),
        DeclareLaunchArgument('max_cams', default_value='2'),
        OpaqueFunction(function=launch_setup),
    ])


