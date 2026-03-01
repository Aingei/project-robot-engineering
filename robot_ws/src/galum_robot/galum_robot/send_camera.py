#!/usr/bin/env python3
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from rclpy import qos
import subprocess
import re

class PiCameraSender(Node):
    def __init__(self):
        super().__init__('pi_camera_sender')
       
        self.publisher_april = self.create_publisher(CompressedImage, '/camera/stream', qos_profile=qos.qos_profile_sensor_data)
        self.publisher_plot = self.create_publisher(CompressedImage, '/camera/stream/plot', qos_profile=qos.qos_profile_sensor_data)
        
        self.get_logger().info("Scanning for cameras by NAME...")

        # -------------------------------------------------------------
        # 🔥 CONFIG
        # -------------------------------------------------------------
        target_cam1_name = "USB Camer "       # Keyword for Camera 1 (AprilTag)
        target_cam2_name = "C922"             # Keyword for Camera 2 (YOLO Plot)

        # ── RESOLUTION & FPS CONFIG ───────────────────────────────────
        self.CAM_WIDTH   = 320    # ลดจาก 640
        self.CAM_HEIGHT  = 240    # ลดจาก 480
        self.TIMER_HZ    = 0.1   # วินาทีต่อเฟรม
        #   0.05 = 20 FPS  (เดิม)
        #   0.10 = 10 FPS  ← แนะนำ
        #   0.15 =  7 FPS
        #   0.20 =  5 FPS
        # ─────────────────────────────────────────────────────────────

        idx1 = self.get_camera_index_by_name(target_cam1_name)
        idx2 = self.get_camera_index_by_name(target_cam2_name)

        if idx1 is None or idx2 is None:
            self.get_logger().warn("Could not find cameras by name. Falling back to index scanning.")
            available = self.scan_all_indices()
            if idx1 is None and len(available) > 0:
                idx1 = available.pop(0)
            if idx2 is None and len(available) > 0:
                idx2 = available.pop(0)

        if idx1 is None: idx1 = 0
        if idx2 is None: idx2 = 2

        if idx1 == idx2:
            self.get_logger().error(f"Conflict: Both cameras assigned to index {idx1}. Shifting Cam2.")
            idx2 = idx1 + 1

        self.get_logger().info(f"Final Assignment: Cam1({target_cam1_name}) -> /dev/video{idx1}, Cam2({target_cam2_name}) -> /dev/video{idx2}")

        self.cap1 = cv2.VideoCapture(idx1)
        self.cap2 = cv2.VideoCapture(idx2)
        
        self.setup_camera(self.cap1)
        self.setup_camera(self.cap2)
        
        self.timer = self.create_timer(self.TIMER_HZ, self.timer_callback)
        self.get_logger().info(f"Dual Camera Node Started | {self.CAM_WIDTH}x{self.CAM_HEIGHT} @ {1/self.TIMER_HZ:.0f} FPS")

    def get_camera_index_by_name(self, target_name):
        try:
            output = subprocess.check_output("v4l2-ctl --list-devices", shell=True).decode("utf-8")
            lines = output.splitlines()
            current_camera_name = ""
            
            for line in lines:
                if not line.startswith("\t") and not line.startswith(" "):
                    current_camera_name = line
                else:
                    if target_name.lower() in current_camera_name.lower():
                        device_path = line.strip()
                        match = re.search(r'/dev/video(\d+)', device_path)
                        if match:
                            index = int(match.group(1))
                            if self.verify_camera(index):
                                self.get_logger().info(f"Found '{target_name}' at index {index}")
                                return index
        except Exception as e:
            self.get_logger().error(f"Error checking camera names: {e}")
            self.get_logger().error("Make sure v4l-utils is installed: sudo apt install v4l-utils")
        return None

    def scan_all_indices(self, limit=10):
        valid = []
        for i in range(limit):
            if self.verify_camera(i):
                valid.append(i)
        return valid

    def verify_camera(self, index):
        cap = cv2.VideoCapture(index)
        if cap.isOpened():  
            ret, _ = cap.read()
            cap.release()
            return ret
        return False

    def setup_camera(self, cap):
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.CAM_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.CAM_HEIGHT)
            cap.set(cv2.CAP_PROP_FPS,          1.0 / self.TIMER_HZ)  # บอกกล้องโดยตรง
            cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)                     # ไม่สะสม buffer

    def timer_callback(self):
        if self.cap1.isOpened():
            ret1, frame1 = self.cap1.read()
            if ret1:
                self.send_frame(self.publisher_april, frame1)
            
        if self.cap2.isOpened():
            ret2, frame2 = self.cap2.read()
            if ret2:
                self.send_frame(self.publisher_plot, frame2)
    
    def send_frame(self, publisher, frame):
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = buffer.tobytes()
        publisher.publish(msg)

def main():
    rclpy.init()
    node = PiCameraSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if hasattr(node, 'cap1'): node.cap1.release()
        if hasattr(node, 'cap2'): node.cap2.release()
        rclpy.shutdown()

if __name__ == '__main__':
    main()