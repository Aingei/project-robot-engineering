#!/usr/bin/env python3
# Save as: robot_master.py (Run on Pi)
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import math
import time

# Define States
STATE_SEARCH = 0      # หา Tag
STATE_APPROACH = 1    # เข้าหา Tag
STATE_GOTO_FIRST = 2  # เดินขนานแปลง ไปยังต้นแรก (40cm)
STATE_WAIT = 99       # ถึงต้นแรกแล้ว หยุดรอ

class RobotMaster(Node):
    def __init__(self):
        super().__init__('robot_master')

        # ==========================================
        # 1. CONFIGURATION SECTION (ตั้งค่าตรงนี้)
        # ==========================================
        # ประกาศตัวแปรเพื่อให้ปรับเปลี่ยนได้ง่าย (Default Values)
        self.declare_parameter('wheel_radius', 0.05)      # รัศมีล้อ (เมตร)
        self.declare_parameter('ticks_per_rev', 7436)     # Ticks ต่อ 1 รอบ
        self.declare_parameter('walk_speed', 0.4)         # ความเร็วเดิน (m/s)
        self.declare_parameter('stop_tag_size', 140)      # ขนาด Tag ที่จะให้หยุด (px)
        self.declare_parameter('kp_tag', 0.15)            # ค่าความไวในการเลี้ยวหา Tag
        self.declare_parameter('kp_row', 0.002)           # ค่าความไวในการเลี้ยวตามร่อง

        # ดึงค่ามาใช้ในตัวแปร
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value
        self.walk_speed = self.get_parameter('walk_speed').value
        self.stop_tag_size = self.get_parameter('stop_tag_size').value
        self.kp_tag = self.get_parameter('kp_tag').value
        self.kp_row = self.get_parameter('kp_row').value

        # คำนวณค่าคงที่ (เมตรต่อ 1 Tick) ไว้เลย
        self.m_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        
        self.get_logger().info(f"Config Loaded: Radius={self.wheel_radius}m, Ticks={self.ticks_per_rev}")
        # ==========================================

        # --- Variables ---
        self.state = STATE_SEARCH
        self.first_plant_dist = 0.0 # เดี๋ยวรับค่าจาก Tag (เช่น 40)
        self.plant_gap = 0.0        
        self.plant_interval = 0.0   
        
        # Vision Data (รับจาก PC)
        self.tag_found = False
        self.tag_error = 0.0
        self.tag_size = 0.0
        self.row_error = 0.0
        
        # Encoder Data
        self.total_distance = 0.0
        self.prev_ticks = [0,0,0,0]
        self.first_run = True

        # --- Pub/Sub ---
        self.cmd_pub = self.create_publisher(Twist, "/galum/cmd_move/rpm", 10)
        
        # รับ Packet รวมจาก PC
        self.create_subscription(Float32MultiArray, "/galum/vision_packet", self.vision_cb, 10)
        # รับ Encoder
        self.create_subscription(Twist, "/galum/encoder", self.encoder_cb, 10)
        
        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Master Robot Started. Waiting for PC Vision...")

    def vision_cb(self, msg):
        # Unpack Data: [TagFound, TagErr, TagSize, RowErr, FirstDist, Gap, Interval]
        d = msg.data
        self.tag_found = (d[0] == 1.0)
        self.tag_error = d[1]
        self.tag_size  = d[2]
        self.row_error = d[3]
        
        # ถ้าอยู่ในโหมด APPROACH และเจอค่า 40115 ให้จำไว้เลย
        if self.state == STATE_APPROACH and d[4] > 0:
            self.first_plant_dist = d[4] / 100.0 # แปลง cm -> m
            self.plant_gap = d[5]
            self.plant_interval = d[6] / 100.0

    def encoder_cb(self, msg):
        # Logic นับระยะทาง (ใช้ตัวแปร self.m_per_tick ที่คำนวณจาก Config ด้านบน)
        curr = [msg.linear.x, msg.linear.y, msg.angular.x, msg.angular.y]
        if self.first_run:
            self.prev_ticks = curr; self.first_run = False; return
        
        d = 0.0
        for i in range(4):
            diff = curr[i] - self.prev_ticks[i]
            if i==0 or i==3: diff = -diff
            d += diff * self.m_per_tick  # <--- ใช้ตรงนี้
        
        self.total_distance += (d/4.0)
        self.prev_ticks = curr

    def control_loop(self):
        left_rpm = 0.0
        right_rpm = 0.0
        
        # =========================================
        # STATE 1: SEARCH (หมุนหา Tag)
        # =========================================
        if self.state == STATE_SEARCH:
            if self.tag_found:
                self.get_logger().info("Tag Found! -> APPROACH")
                self.state = STATE_APPROACH
            else:
                # หมุนรอบตัวช้าๆ
                left_rpm = -30.0
                right_rpm = 30.0

        # =========================================
        # STATE 2: APPROACH (วิ่งเข้าหา Tag)
        # =========================================
        elif self.state == STATE_APPROACH:
            if not self.tag_found:
                self.state = STATE_SEARCH
            else:
                # ใช้ config stop_tag_size
                if self.tag_size > self.stop_tag_size:
                    self.get_logger().info(f"Arrived! Code: Dist={self.first_plant_dist}m")
                    self.total_distance = 0.0
                    self.state = STATE_GOTO_FIRST
                    self.send_speed(0,0)
                    return
                
                # PID เข้าหา Tag (ใช้ config kp_tag)
                turn = self.tag_error * self.kp_tag
                fwd = 80.0 # หรือจะเปลี่ยนเป็น self.walk_speed แปลงเป็น RPM ก็ได้
                left_rpm = fwd - turn
                right_rpm = fwd + turn

        # =========================================
        # STATE 3: GOTO FIRST PLANT (เดินขนานแปลง ไปต้นแรก)
        # =========================================
        elif self.state == STATE_GOTO_FIRST:
            if abs(self.total_distance) >= self.first_plant_dist:
                self.get_logger().info("REACHED FIRST PLANT! Stopping.")
                self.state = STATE_WAIT
            else:
                # เดินขนานแปลง (ใช้ config kp_row)
                rotation = (self.row_error * self.kp_row) * -1.0
                
                v_l = self.walk_speed + rotation
                v_r = self.walk_speed - rotation
                
                # คำนวณ RPM โดยใช้ radius จาก config
                left_rpm = (v_l / (2*math.pi*self.wheel_radius)) * 60
                right_rpm = (v_r / (2*math.pi*self.wheel_radius)) * 60
                
                print(f"To First Plant: {self.total_distance:.2f}/{self.first_plant_dist} m", end='\r')

        # =========================================
        # STATE 4: WAIT
        # =========================================
        elif self.state == STATE_WAIT:
            left_rpm = 0.0
            right_rpm = 0.0

        self.send_speed(left_rpm, right_rpm)

    def send_speed(self, l, r):
        msg = Twist()
        msg.linear.x, msg.angular.x = float(l), float(l)
        msg.linear.y, msg.angular.y = float(r), float(r)
        self.cmd_pub.publish(msg)

def main():
    rclpy.init()
    node = RobotMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()