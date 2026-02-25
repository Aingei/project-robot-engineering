#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from rclpy import qos
import math

# Define States
STATE_GOTO_FIRST = 2  # เดินขนานแปลง (เราจะเริ่มที่นี่เลย)
STATE_WAIT = 99       # จบ

class RobotMasterTest(Node):
    def __init__(self):
        super().__init__('robot_master_test')

        # --- Config ---
        self.wheel_radius = 0.05
        self.ticks_per_rev = 7436
        self.m_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        
        # ปรับความเร็วเดินตรงนี้
        self.walk_speed = 0.4        
        
        # 🔥 ปรับความไวในการเลี้ยงร่องตรงนี้ (ถ้าส่ายไปมาให้ลดลง / ถ้าไม่เลี้ยวสู้ให้เพิ่มขึ้น)
        self.kp_row = 0.003          

        # --- Variables ---
        # 🔥 บังคับเริ่มที่โหมดเดินขนานเลย
        self.state = STATE_GOTO_FIRST
        
        # 🔥 กำหนดระยะทางที่จะทดสอบเดิน (เช่น 2.0 เมตร)
        self.target_dist = 2.0       
        
        # Vision & Encoder Data
        self.yolo_lat_err = 0.0      
        self.total_dist = 0.0
        self.prev_ticks = [0,0,0,0]
        self.first_run = True

        # --- Pub/Sub ---
        self.cmd_pub = self.create_publisher(Twist, "/galum/cmd_move/rpm", 10)
        
        # รับ Vision Packet
        self.create_subscription(Float32MultiArray, "/galum/vision_packet", self.vision_cb, qos_profile=qos.qos_profile_sensor_data)
        # รับ Encoder (ใช้ Sensor Data เพื่อความไว)
        self.create_subscription(Twist, "/galum/encoder", self.encoder_cb, qos.qos_profile_sensor_data)
        
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"TEST MODE STARTED: Walking {self.target_dist}m parallel to row...")

    def vision_cb(self, msg):
        # [Found, TagErr, TagSize, LatErr, ...]
        d = msg.data
        self.yolo_lat_err = d[3] # รับค่า Error ร่องแปลง

    def encoder_cb(self, msg):
        curr = [msg.linear.x, msg.linear.y, msg.angular.x, msg.angular.y]
        if self.first_run:
            self.prev_ticks = curr; self.first_run = False; return
        
        d = 0.0
        for i in range(4):
            diff = curr[i] - self.prev_ticks[i]
            if i==0 or i==3: diff = -diff
            d += diff * self.m_per_tick
        self.total_dist += (d/4.0)
        self.prev_ticks = curr

    def control_loop(self):
        left_rpm = 0.0
        right_rpm = 0.0

        # =========================================
        # STATE GOTO FIRST (เดินขนานแปลง)
        # =========================================
        if self.state == STATE_GOTO_FIRST:
            # เช็คระยะทางว่าครบกำหนดหรือยัง
            if abs(self.total_dist) >= self.target_dist:
                self.get_logger().info("TEST COMPLETE: Reached Target Distance.")
                self.state = STATE_WAIT
            else:
                # 🔥 คำนวณการเลี้ยวจาก YOLO Lat Error
                # สูตร: ยิ่งห่างเส้นกลาง (lat_err เยอะ) ยิ่งต้องเลี้ยวแรง
                turn = (self.yolo_lat_err * self.kp_row) * -1.0 
                
                # คำนวณความเร็ว
                v_l = self.walk_speed + turn
                v_r = self.walk_speed - turn
                
                # แปลง m/s -> RPM
                left_rpm = (v_l / (2*math.pi*self.wheel_radius)) * 60
                right_rpm = (v_r / (2*math.pi*self.wheel_radius)) * 60
                
                # Debug ดูค่า (เปิดคอมเมนต์ถ้าอยากดู Realtime)
                # print(f"Dist: {abs(self.total_dist):.2f}/{self.target_dist}m | Err: {self.yolo_lat_err:.1f} | RPM: {left_rpm:.0f}/{right_rpm:.0f}", end='\r')

        # =========================================
        # STATE WAIT: จบ
        # =========================================
        elif self.state == STATE_WAIT:
            left_rpm, right_rpm = 0.0, 0.0

        self.send_rpm(left_rpm, right_rpm)

    def send_rpm(self, l, r):
        msg = Twist()
        msg.linear.x, msg.angular.x = float(l), float(l)
        msg.linear.y, msg.angular.y = float(r), float(r)
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = RobotMasterTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()