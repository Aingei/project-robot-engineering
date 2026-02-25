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

        # Config
        self.wheel_radius = 0.05
        self.ticks_per_rev = 7436
        self.m_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        
        self.walk_speed = 0.4        
        
        # 🔥 แก้ 1: ลดขนาดเหลือ 80 (เพราะภาพเราเล็ก 320x240)
        self.stop_tag_size = 110      
        
        self.kp_tag = 0.15           
        self.kp_row = 0.003          

        # Variables
        self.state = STATE_SEARCH
        self.target_dist = 0.0       
        
        # Vision Data
        self.tag_found = False
        self.tag_err = 0.0
        self.tag_size = 0.0
        self.yolo_lat_err = 0.0      
        
        # Encoder & Timer
        self.total_dist = 0.0
        self.prev_ticks = [0,0,0,0]
        self.first_run = True
        self.pause_start_time = 0.0
        
        # 🔥 เพิ่มตัวแปรจำค่าล่าสุด กันป้ายหลุด
        self.last_tag_size = 0.0
        
        self.turn_start_time = 0.0

        # ROS
        self.cmd_pub = self.create_publisher(Twist, "/galum/cmd_move/rpm", 10)
        self.create_subscription(Float32MultiArray, "/galum/vision_packet", self.vision_cb, qos_profile=qos.qos_profile_sensor_data)
        self.create_subscription(Twist, "/galum/encoder", self.encoder_cb, qos_profile=qos.qos_profile_sensor_data)
        
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Robot Master Started (Smart Stop Fix)")

    def vision_cb(self, msg):
        d = msg.data
        self.tag_found = (d[0] == 1.0)
        self.tag_err = d[1]
        self.tag_size = d[2]
        self.yolo_lat_err = d[3] 
        
        if self.tag_found:
            self.last_tag_size = self.tag_size # จำค่าล่าสุดไว้

        if self.state <= STATE_APPROACH and d[4] > 0:
            self.target_dist = d[4] / 100.0 

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
                self.state = STATE_APPROACH
            else:
                left_rpm, right_rpm = -30.0, 30.0

        # 2. APPROACH
        elif self.state == STATE_APPROACH:
            # เงื่อนไขหยุด 1: ป้ายใหญ่พอ
            condition_1 = self.tag_size > self.stop_tag_size
            
            # 🔥 เงื่อนไขหยุด 2: ป้ายหาย แต่เมื่อกี้อยู่ใกล้มากแล้ว (Blind Stop)
            condition_2 = (not self.tag_found) and (self.last_tag_size > 60)

            if condition_1 or condition_2:
                self.get_logger().info(f"Arrived! (Size: {self.tag_size} / Last: {self.last_tag_size})")
                self.pause_start_time = time.time()
                self.state = STATE_PAUSE
                self.send_rpm(0, 0)
                return
            
            elif not self.tag_found:
                # ถ้าไม่เจอและยังอยู่ไกล -> กลับไปหาใหม่
                self.state = STATE_SEARCH
            
            else:
                # PID เข้าหา
                turn = self.tag_err * self.kp_tag
                left_rpm = 60.0 - turn
                right_rpm = 60.0 + turn
                
                # Debug ดูขนาดป้าย
                print(f"Approaching... Size: {self.tag_size:.1f} / {self.stop_tag_size}", end='\r')

        # 3. PAUSE (หยุด 2 วิ)
        elif self.state == STATE_PAUSE:
            left_rpm, right_rpm = 0.0, 0.0
            if time.time() - self.pause_start_time > 2.0:
                self.get_logger().info("Pause Done -> START TURNING")
                
                # 🔥 ต้องเพิ่มบรรทัดนี้! เพื่อเริ่มนับวินาทีที่ 0
                self.turn_start_time = time.time() 
                
                self.state = STATE_TURNL
                
        elif self.state == STATE_TURNL:
            # คำนวณเวลาที่ผ่านไป
            elapsed = time.time() - self.turn_start_time
            
            # เช็คว่าครบ 5 วินาทีหรือยัง
            if elapsed > 7.0:
                self.get_logger().info("Turn Done -> Go YOLO")
                self.total_dist = 0.0 
                self.state = STATE_GOTO_FIRST 
            else:
                # 🔥 แก้สูตรหมุนซ้ายให้ง่ายและนิ่ง: ล้อซ้ายถอย ล้อขวาเดินหน้า
                turn_speed_rpm = 30.0 
                left_rpm = -turn_speed_rpm
                right_rpm = turn_speed_rpm
                
                # Debug เวลาที่เหลือ
                # print(f"Turning... {elapsed:.1f}/5.0 s", end='\r')
                

        # 4. GOTO FIRST
        elif self.state == STATE_GOTO_FIRST:
            if abs(self.total_dist) >= self.target_dist:
                self.get_logger().info("REACHED PLANT! Stopping.")
                self.state = STATE_WAIT
            else:
                turn = (self.yolo_lat_err * self.kp_row) * -1.0 
                max_turn = self.walk_speed * 0.8 
                if turn > max_turn: turn = max_turn
                if turn < -max_turn: turn = -max_turn
                
                v_l = self.walk_speed + turn
                v_r = self.walk_speed - turn
                left_rpm = (v_l / (2*math.pi*self.wheel_radius)) * 60
                right_rpm = (v_r / (2*math.pi*self.wheel_radius)) * 60

        # 5. WAIT
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