#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math
import cv2
import pupil_apriltags
import numpy as np
from galum_robot.utilize import *
from galum_robot.controller import *

class Test(Node):
    def __init__(self):
        super().__init__('test')

        # === ส่วนตั้งค่าหุ่นยนต์ ===
        self.send_robot_speed = self.create_publisher(Twist, "/galum/cmd_move/rpm", 10 )
        self.create_subscription(Twist, "/galum/imu_angle", self.get_robot_angle, 10)
        
        # === ส่วนตั้งค่ากล้องและ AprilTag ===
        # หมายเหตุ: ลองเปลี่ยนเลข 0, 1, 2 จนกว่าจะเจอกล้องที่ถูกต้อง
        self.cap = cv2.VideoCapture(1) 
        self.detector = pupil_apriltags.Detector(families='tagStandard52h13')
        
        # !! สำคัญ !! การคำนวณระยะต้องใช้ค่าเหล่านี้
        self.tag_size = 0.05    # ขนาดจริงของ Tag (หน่วยเมตร) เช่น 5 cm = 0.05
        # ค่ากล้อง (fx, fy, cx, cy) ถ้าไม่ได้ Calibrate ให้ใช้ค่าประมาณนี้ (สำหรับกล้อง WebCam ทั่วไป)
        self.camera_params = [600, 600, 320, 240] 

        # === ตั้งค่าการเดิน ===
        self.moveSpeed = 10.0      # ความเร็วเดิน
        self.walk_time = 20.0      # เพิ่มเวลาเผื่อไว้ เพราะเราจะหยุดด้วย Tag แทน
        self.timer = self.create_timer(0.05, self.loop)

        self.previous_time = time.time()
        self.stopped = False
        self.tag_found = False # ตัวแปรเช็คว่าเจอ Tag หรือยัง

        # PID Variables
        self.yaw = 0.0
        self.yaw_setpoint = 0.0
        self.maxSpeed : float = 1023.0
        self.motor1Speed = 0
        self.motor2Speed = 0
        self.motor3Speed = 0
        self.motor4Speed = 0
        
        self.controller = Controller(kp = 1.0, ki = 0.05, kd = 0.001, errorTolerance= To_Radians(0.5), i_min= -1, i_max= 1)

        self.get_logger().info('AUTO WALK START - CAMERA ACTIVE')

    def get_robot_angle(self, msg):
        self.yaw = WrapRads(To_Radians(msg.linear.x))

    def decode_cabbage_data(self, tag_id):
        s = str(tag_id).zfill(5)
        if len(s) != 5: return None
        data = {
            "id": tag_id,
            "planting_dist": int(s[0:2]),
            "gap": {'1':5, '2':10, '3':15 , '4':20 , '5':25}.get(s[2], 0),
            "interval": int(s[3:5])
        }
        return data

    def loop(self):
        elapsed = time.time() - self.previous_time
        
        # 1. อ่านภาพจากกล้อง
        ret, frame = self.cap.read()
        if ret:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # 2. ตรวจจับ Tag และขอ Pose (ระยะทาง)
            detections = self.detector.detect(
                gray, 
                estimate_tag_pose=True, 
                camera_params=self.camera_params, 
                tag_size=self.tag_size
            )

            # 3. ถ้าเจอ Tag (และยังไม่เคยเจอมาก่อน)
            if len(detections) > 0:
                tag = detections[0] # เอาตัวแรกที่เจอ
                
                # ถ้ายังไม่เคยเจอมาก่อน ให้แสดงผลและสั่งหยุด
                if not self.tag_found:
                    self.tag_found = True
                    
                    # ดึงค่าระยะทาง (แกน Z คือระยะห่างจากหน้ากล้อง)
                    distance_meters = tag.pose_t[2][0] 
                    decoded_info = self.decode_cabbage_data(tag.tag_id)
                    
                    self.get_logger().info(f"!!! FOUND TAG ID: {tag.tag_id} !!!")
                    self.get_logger().info(f"Distance: {distance_meters:.3f} meters")
                    if decoded_info:
                        self.get_logger().info(f"Decoded: {decoded_info}")

                # (Optional) วาดรูปเพื่อ Debug
                # pts = tag.corners.reshape((-1, 1, 2)).astype(int)
                # cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
                # cv2.imshow("Robot Camera", frame)
                # cv2.waitKey(1)
            else:
                # ถ้าไม่เจอ Tag ให้เคลียร์ค่าได้ (หรือจะจำค่าเดิมไว้ก็ได้)
                pass

        # 4. Logic การเดิน
        # เดินต่อเมื่อ: เวลายังไม่หมด AND ยังไม่เจอ Tag
        if elapsed < self.walk_time and not self.tag_found:
            
            # PID คำนวณทิศทาง
            error = WrapRads(self.yaw_setpoint - self.yaw)
            rotation = self.controller.Calculate(error)
            
            # คำนวณความเร็วมอเตอร์
            self.motor1Speed = (self.moveSpeed + rotation) * self.maxSpeed 
            self.motor2Speed = (self.moveSpeed - rotation) * self.maxSpeed
            self.motor3Speed = (self.moveSpeed + rotation) * self.maxSpeed 
            self.motor4Speed = (self.moveSpeed - rotation) * self.maxSpeed
            
            self.sendData()
            
        else:
            # หยุดเดิน (เมื่อหมดเวลา หรือ เจอ Tag แล้ว)
            self.motor1Speed = 0.0
            self.motor2Speed = 0.0
            self.motor3Speed = 0.0
            self.motor4Speed = 0.0
            self.sendData()

            if not self.stopped:
                if self.tag_found:
                    self.get_logger().info('STOPPED: Target Found')
                else:
                    self.get_logger().info('STOPPED: Time Limit')
                self.stopped = True

    def sendData(self):
        motorspeed_msg = Twist()
        motorspeed_msg.linear.x = float(self.motor1Speed) 
        motorspeed_msg.linear.y = float(self.motor2Speed) 
        motorspeed_msg.angular.x = float(self.motor3Speed) 
        motorspeed_msg.angular.y = float(self.motor4Speed) 
        self.send_robot_speed.publish(motorspeed_msg)

    # เพิ่มตัวทำลาย Node เพื่อปิดกล้องให้เรียบร้อยเมื่อกด Ctrl+C
    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = Test()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()