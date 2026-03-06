#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np
import pupil_apriltags
import math

class AprilTagFollower(Node):
    def __init__(self):
        super().__init__('apriltag_follower')

        # --- Publishers (ส่งคำสั่งไป ESP32) ---
        self.send_robot_speed = self.create_publisher(
            Twist, "/galum/cmd_move/rpm", 10)
        
        # --- Subscribers (รับภาพจาก Pi) ---
        self.create_subscription(
            CompressedImage, '/camera/stream', self.process_frame, qos_profile=qos.qos_profile_sensor_data)

        # --- Setup Detector ---
        self.detector = pupil_apriltags.Detector(families='tagStandard52h13')

        # --- Robot Config (ตั้งค่าตามหุ่นจริง) ---
        self.wheel_radius = 0.05        # 50mm
        self.track_width = 0.20         # ระยะห่างล้อ (เมตร) วัดให้แม่นเพื่อการเลี้ยวที่ดี
        
        # --- Tuning Parameters (จูนตรงนี้) ---
        self.forward_rpm = 80.0         # ความเร็วเดินหน้า (RPM)
        self.turn_kp = 0.15             # ค่าความไวในการเลี้ยว (ถ้าส่ายให้ลด, ถ้าเลี้ยวไม่ทันให้เพิ่ม)
        self.stop_threshold = 120       # ขนาด Tag (pixel) ที่จะให้หยุด (ค่ายิ่งมาก ยิ่งหยุดใกล้)
        
        # --- State Variables ---
        self.searching = True           # True = หมุนหา, False = เจอแล้วเดินตาม
        self.no_tag_count = 0           # ตัวนับเวลาไม่เจอ Tag

        self.get_logger().info("AprilTag Follower Started...")

    def process_frame(self, msg):
        # 1. แปลงภาพ
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None: return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        height, width = gray.shape
        center_x = width // 2

        # 2. ตรวจจับ AprilTag
        detections = self.detector.detect(gray)

        left_rpm = 0.0
        right_rpm = 0.0

        if detections:
            self.no_tag_count = 0
            # เลือก Tag ที่ใหญ่ที่สุด (ใกล้ที่สุด)
            tag = max(detections, key=lambda x: x.decision_margin)
            
            # --- ดึงพิกัดมุมของ Tag ---
            pts = tag.corners.reshape((-1, 1, 2)).astype(int)
            
            # คำนวณจุดกึ่งกลาง Tag (cx, cy)
            tag_cx = int(tag.center[0])
            tag_cy = int(tag.center[1])
            
            # คำนวณ "ความกว้าง" ของ Tag ในภาพ (ใช้ประเมินระยะห่าง)
            # เอาผลต่างของมุมขวาบน - มุมซ้ายบน
            tag_width_px = abs(pts[1][0][0] - pts[0][0][0])

            # --- Logic การควบคุม (PID ง่ายๆ) ---
            
            # 1. การเลี้ยว (Heading Control)
            # error คือ ระยะห่างจากจุดกึ่งกลางภาพ
            error_x = center_x - tag_cx 
            # ถ้า error เป็น + (Tag อยู่ซ้าย) -> ต้องหมุนซ้าย (ล้อขวาแรงกว่า)
            # ถ้า error เป็น - (Tag อยู่ขวา) -> ต้องหมุนขวา (ล้อซ้ายแรงกว่า)
            turn_adjust = error_x * self.turn_kp

            # 2. การเดินหน้า (Distance Control)
            if tag_width_px < self.stop_threshold:
                # ยังไกลอยู่ -> เดินหน้า
                move_cmd = self.forward_rpm
                status = "APPROACHING"
            else:
                # ใกล้พอแล้ว -> หยุด
                move_cmd = 0.0
                status = "STOPPED (Arrived)"
                
            # ผสมความเร็ว (Differential Drive Mixing)
            # ล้อซ้าย = เดินหน้า - เลี้ยว
            # ล้อขวา = เดินหน้า + เลี้ยว
            left_rpm = move_cmd - turn_adjust
            right_rpm = move_cmd + turn_adjust

            # --- วาดภาพ Debug ---
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            cv2.circle(frame, (tag_cx, tag_cy), 5, (0, 0, 255), -1)
            cv2.line(frame, (center_x, 0), (center_x, height), (255, 255, 0), 1) # เส้นกลางจอ
            
            # แสดงข้อมูลบนจอ
            info_text = f"ID:{tag.tag_id} W:{tag_width_px:.0f}px Err:{error_x} RPM:{int(left_rpm)}|{int(right_rpm)}"
            cv2.putText(frame, info_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            cv2.putText(frame, status, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        else:
            # --- กรณีไม่เจอ Tag ---
            self.no_tag_count += 1
            if self.no_tag_count > 10: # ถ้าไม่เจอ 10 เฟรมติดกัน
                # ให้หยุด (หรือจะสั่งให้หมุนรอบตัวเพื่อหาก็ได้)
                left_rpm = 0.0
                right_rpm = 0.0
                cv2.putText(frame, "SEARCHING...", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # 3. ส่งคำสั่งไปหุ่นยนต์
        self.send_speed(left_rpm, right_rpm)

        # 4. แสดงผล
        cv2.imshow("AprilTag Follower", frame)
        cv2.waitKey(1)

    def send_speed(self, left, right):
        # จำกัดความเร็วไม่ให้เกิน Hardware Limit (เช่น 150)
        max_limit = 150.0
        left = max(min(left, max_limit), -max_limit)
        right = max(min(right, max_limit), -max_limit)

        msg = Twist()
        msg.linear.x = float(left)    # FL
        msg.angular.x = float(left)   # RL
        msg.linear.y = float(right)   # FR
        msg.angular.y = float(right)  # RR
        self.send_robot_speed.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = AprilTagFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.send_speed(0,0) # หยุดหุ่นก่อนปิด
        node.destroy_node()
        cv2.destroyAllWindows()
        rclpy.shutdown()

if __name__ == '__main__':
    main()