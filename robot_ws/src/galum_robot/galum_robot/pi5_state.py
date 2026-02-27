#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32 
from rclpy import qos
import math
import time

# --- States ---
STATE_SEARCH = 0      
STATE_APPROACH = 1    
STATE_PAUSE = 15      
STATE_TURNL = 3
STATE_FORWARD_AFTER_TURN = 5  # เพิ่ม State ใหม่: เดินหน้าหลังเลี้ยว
STATE_ALIGN_ROW = 4   
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
        self.stop_distance_threshold = 0.30 
        
        # PID Gains
        self.kp_tag = 0.15           
        self.kp_row = 0.003          

        # Variables
        self.state = STATE_SEARCH
        self.target_m = 0.5 
        
        # Vision Data
        self.tag_found = False
        self.tag_err = 0.0
        self.tag_size = 0.0
        self.yolo_lat_err = 0.0
        self.yolo_found = False
        self.current_z_dist = 0.0
        
        # Encoder & Timer
        self.total_dist = 0.0
        self.prev_ticks = [0,0,0,0]
        self.first_run = True
        self.pause_start_time = 0.0
        self.turn_start_time = 0.0
        self.forward_start_time = 0.0 # 🔥 เพิ่มตัวแปรจับเวลาเดินหน้า

        # ROS Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, "/galum/cmd_move/rpm", 10)
        self.create_subscription(Float32MultiArray, "/galum/vision_packet", self.vision_cb, qos_profile=qos.qos_profile_sensor_data)
        self.create_subscription(Twist, "/galum/encoder", self.encoder_cb, qos_profile=qos.qos_profile_sensor_data)
        
        # Debug Plot
        self.debug_err_pub = self.create_publisher(Float32, "/debug/error_input", 10)
        self.debug_pid_pub = self.create_publisher(Float32, "/debug/pid_output", 10)
        
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Robot Master: Turn -> Forward 3s -> Scan")

    def vision_cb(self, msg):
        d = msg.data
        self.tag_found = (d[0] == 1.0)
        self.tag_err = d[1]
        self.tag_size = d[2]
        self.yolo_lat_err = d[3] 
        
        if self.tag_found and len(d) > 4:
            dist_from_tag = d[4] 
            if self.state == STATE_SEARCH and dist_from_tag > 0:
                self.target_m = dist_from_tag / 100.0 

        if len(d) > 7:
            self.current_z_dist = d[7]
            
        if len(d) > 8:
            self.yolo_found = (d[8] == 1.0)
        else:
            self.yolo_found = True 

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
        pid_out = 0.0
        error_in = 0.0
        
        # 1. SEARCH
        if self.state == STATE_SEARCH:
            if self.tag_found:
                self.state = STATE_APPROACH
            else:
                left_rpm, right_rpm = -30.0, 30.0

        # 2. APPROACH
        elif self.state == STATE_APPROACH:
            stop_condition = (self.current_z_dist > 0.0) and (self.current_z_dist < self.stop_distance_threshold)
            if self.tag_found and stop_condition:
                self.get_logger().info(f"Arrived Z: {self.current_z_dist:.2f}m")
                self.pause_start_time = time.time()
                self.state = STATE_PAUSE
                self.send_rpm(0, 0)
                return
            elif not self.tag_found:
                self.state = STATE_SEARCH
            else:
                error_in = self.tag_err
                pid_out = self.tag_err * self.kp_tag 
                left_rpm = 60.0 - pid_out
                right_rpm = 60.0 + pid_out

        # 3. PAUSE
        elif self.state == STATE_PAUSE:
            left_rpm, right_rpm = 0.0, 0.0
            if time.time() - self.pause_start_time > 2.0:
                self.turn_start_time = time.time() 
                self.state = STATE_TURNL
                
        # 4. TURN LEFT (6 วินาที)
        elif self.state == STATE_TURNL:
            elapsed = time.time() - self.turn_start_time
            if elapsed < 8.0:
                left_rpm, right_rpm = -30.0, 30.0
            else:
                # ครบ 6 วิ -> ไปเดินหน้าต่อ
                self.get_logger().info("Turn Done -> Moving Forward 3s")
                self.forward_start_time = time.time() # เริ่มจับเวลาเดินหน้า
                self.state = STATE_FORWARD_AFTER_TURN

        # 🔥 5. FORWARD 3 SEC (State ใหม่)
        elif self.state == STATE_FORWARD_AFTER_TURN:
            elapsed = time.time() - self.forward_start_time
            
            if elapsed < 5.0: # เดินหน้า 3 วินาที
                # เดินหน้าด้วยความเร็วปกติ (0.4 m/s -> RPM คำนวณเอา)
                fwd_rpm = (self.walk_speed / (2*math.pi*self.wheel_radius)) * 60
                left_rpm, right_rpm = fwd_rpm, fwd_rpm
                
            else:
                # ครบ 3 วิ -> เริ่มเช็คหาแปลง
                if self.yolo_found:
                    self.get_logger().info("Forward Done & Found Row -> ALIGN")
                    self.total_dist = 0.0 
                    self.state = STATE_ALIGN_ROW
                else:
                    self.get_logger().info("Forward Done but NO ROW -> Scanning...")
                    left_rpm, right_rpm = -10.0, 10.0 # หมุนหา

        # 6. ALIGN ROW
        elif self.state == STATE_ALIGN_ROW:
            align_dist = 0.20 
            if not self.yolo_found:
                left_rpm, right_rpm = 0.0, 0.0 # หยุดถ้ายัดเยียดไม่เห็น
                self.get_logger().info("Lost vision!", once=True)
            else:
                error_in = self.yolo_lat_err
                pid_out = (self.yolo_lat_err * self.kp_row) * -1.0 
                
                max_turn = self.walk_speed * 0.8
                if pid_out > max_turn: pid_out = max_turn
                if pid_out < -max_turn: pid_out = -max_turn
                
                v_l = self.walk_speed + pid_out
                v_r = self.walk_speed - pid_out
                left_rpm = (v_l / (2*math.pi*self.wheel_radius)) * 60
                right_rpm = (v_r / (2*math.pi*self.wheel_radius)) * 60
                
                if abs(self.total_dist) >= align_dist:
                    if abs(self.yolo_lat_err) < 20.0:
                        self.get_logger().info("ALIGNED! -> GO")
                        self.total_dist = 0.0 
                        self.state = STATE_GOTO_FIRST

        # 7. GOTO FIRST
        elif self.state == STATE_GOTO_FIRST:
            if abs(self.total_dist) >= self.target_dist_check():
                self.get_logger().info(f"REACHED TARGET. STOP.")
                self.state = STATE_WAIT
            else:
                if not self.yolo_found:
                    pid_out = 0 
                else:
                    error_in = self.yolo_lat_err
                    pid_out = (self.yolo_lat_err * self.kp_row) * -1.0
                
                max_turn = self.walk_speed * 0.8 
                if pid_out > max_turn: pid_out = max_turn
                if pid_out < -max_turn: pid_out = -max_turn
                
                v_l = self.walk_speed + pid_out
                v_r = self.walk_speed - pid_out
                left_rpm = (v_l / (2*math.pi*self.wheel_radius)) * 60
                right_rpm = (v_r / (2*math.pi*self.wheel_radius)) * 60

        # 8. WAIT
        elif self.state == STATE_WAIT:
            left_rpm, right_rpm = 0.0, 0.0

        self.send_rpm(left_rpm, right_rpm)
        
        msg_err = Float32(); msg_err.data = float(error_in)
        msg_pid = Float32(); msg_pid.data = float(pid_out)
        self.debug_err_pub.publish(msg_err)
        self.debug_pid_pub.publish(msg_pid)

    def target_dist_check(self):
        return self.target_m if self.target_m > 0 else 0.5

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