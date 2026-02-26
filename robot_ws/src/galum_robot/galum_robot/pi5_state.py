#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from rclpy import qos
import math
import time

# Define States
STATE_SEARCH = 0      
STATE_APPROACH = 1    
STATE_PAUSE = 15      
STATE_TURNL = 3
STATE_GOTO_FIRST = 2  
STATE_WAIT = 99       

class RobotMaster(Node):
    def __init__(self):
        super().__init__('robot_master')

        # --- Config ---
        self.wheel_radius = 0.05
        self.ticks_per_rev = 7436
        self.m_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        
        self.walk_speed = 0.4        
        
        # 🔥 แก้ 1: ระยะหยุดห่างจากป้าย (หน่วยเมตร)
        # 0.60 = 60 cm (ปรับตามความเหมาะสม)
        self.stop_distance_threshold = 0.60 
        
        self.kp_tag = 0.15           
        self.kp_row = 0.003          

        # Variables
        self.state = STATE_SEARCH
        
        # 🔥 แก้ 2: ตัวแปรเป้าหมายระยะทาง (Default 0.5 เมตร เผื่อไม่เจอ)
        self.target_m = 0.5 
        
        # Vision Data
        self.tag_found = False
        self.tag_err = 0.0
        self.tag_size = 0.0
        self.yolo_lat_err = 0.0
        
        # 🔥 แก้ 3: ตัวแปรเก็บระยะห่างจริง (Z-Dist)
        self.current_z_dist = 0.0
        
        # Encoder & Timer
        self.total_dist = 0.0
        self.prev_ticks = [0,0,0,0]
        self.first_run = True
        self.pause_start_time = 0.0
        self.turn_start_time = 0.0

        # ROS
        self.cmd_pub = self.create_publisher(Twist, "/galum/cmd_move/rpm", 10)
        self.create_subscription(Float32MultiArray, "/galum/vision_packet", self.vision_cb, qos_profile=qos.qos_profile_sensor_data)
        self.create_subscription(Twist, "/galum/encoder", self.encoder_cb, qos_profile=qos.qos_profile_sensor_data)
        
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Robot Master Started (Z-Dist Stop & Auto Row Dist)")

    def vision_cb(self, msg):
        d = msg.data
        # d = [Found, X_Err, Size, Yolo_Err, PlantDist(4), Gap, Interval, Z_Dist(7)]
        
        self.tag_found = (d[0] == 1.0)
        self.tag_err = d[1]
        self.tag_size = d[2]
        self.yolo_lat_err = d[3] 
        
        # 🔥 1. รับค่าระยะทางที่จะเดิน (จากป้าย ID)
        # d[4] คือค่าที่ PC ถอดรหัสมา (เช่น 40.0)
        if self.tag_found and len(d) > 4:
            dist_from_tag = d[4] 
            # อัพเดทเฉพาะตอนค้นหา เพื่อไม่ให้ค่าโดดไปมาตอนเข้าใกล้
            if self.state == STATE_SEARCH and dist_from_tag > 0:
                self.target_m = dist_from_tag / 100.0 # แปลง cm -> m (0.4m)

        # 🔥 2. รับค่า Z-Distance (ระยะห่างจริงจากป้าย)
        # อยู่ที่ Index 7 (ตัวที่ 8)
        if len(d) > 7:
            self.current_z_dist = d[7]

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
        
        # 1. SEARCH
        if self.state == STATE_SEARCH:
            if self.tag_found:
                self.get_logger().info(f"Tag Found! Will walk: {self.target_m} m")
                self.state = STATE_APPROACH
            else:
                left_rpm, right_rpm = -30.0, 30.0

        # 2. APPROACH (เข้าหาป้าย)
        elif self.state == STATE_APPROACH:
            # 🔥 เงื่อนไขหยุดใหม่: ใช้ระยะจริง (Z) น้อยกว่าที่ตั้งไว้ (เช่น 0.6m)
            # ต้องมากกว่า 0 ด้วย (กันค่าขยะ)
            stop_condition = (self.current_z_dist > 0.0) and (self.current_z_dist < self.stop_distance_threshold)
            
            if self.tag_found and stop_condition:
                self.get_logger().info(f"Arrived! Z-Dist: {self.current_z_dist:.2f} m")
                self.pause_start_time = time.time()
                self.state = STATE_PAUSE
                self.send_rpm(0, 0)
                return
            
            elif not self.tag_found:
                self.state = STATE_SEARCH
            
            else:
                # PID เข้าหา
                turn = self.tag_err * self.kp_tag
                left_rpm = 60.0 - turn
                right_rpm = 60.0 + turn
                # Debug ดูระยะจริง
                # print(f"Appr... Z: {self.current_z_dist:.2f}m / Target: {self.stop_distance_threshold}m", end='\r')

        # 3. PAUSE (หยุด 2 วิ)
        elif self.state == STATE_PAUSE:
            left_rpm, right_rpm = 0.0, 0.0
            if time.time() - self.pause_start_time > 2.0:
                self.get_logger().info("Pause Done -> START TURNING")
                self.turn_start_time = time.time() 
                self.state = STATE_TURNL
                
        # 4. TURN LEFT
        elif self.state == STATE_TURNL:
            elapsed = time.time() - self.turn_start_time
            if elapsed > 7.0: # หมุน 7 วิ
                self.get_logger().info("Turn Done -> RESET DIST -> Go YOLO")
                
                # 🔥 สำคัญ: รีเซ็ตระยะทางเป็น 0 (เริ่มนับต้นแปลง)
                self.total_dist = 0.0 
                
                self.state = STATE_GOTO_FIRST 
            else:
                turn_speed_rpm = 30.0 
                left_rpm = -turn_speed_rpm
                right_rpm = turn_speed_rpm

        # 5. GOTO FIRST (เดินตามร่อง)
        elif self.state == STATE_GOTO_FIRST:
            current_val = abs(self.total_dist)
            
            # 🔥 ใช้เป้าหมายที่อ่านได้จากป้าย (self.target_m)
            if current_val >= self.target_m:
                self.get_logger().info(f"REACHED TARGET ({current_val:.2f} / {self.target_m} m). STOP.")
                self.state = STATE_WAIT
            else:
                # YOLO Lane Keeping
                turn = (self.yolo_lat_err * self.kp_row) * -1.0 
                max_turn = self.walk_speed * 0.8 
                if turn > max_turn: turn = max_turn
                if turn < -max_turn: turn = -max_turn
                
                v_l = self.walk_speed + turn
                v_r = self.walk_speed - turn
                left_rpm = (v_l / (2*math.pi*self.wheel_radius)) * 60
                right_rpm = (v_r / (2*math.pi*self.wheel_radius)) * 60

        # 6. WAIT
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
    node = RobotMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__': main()