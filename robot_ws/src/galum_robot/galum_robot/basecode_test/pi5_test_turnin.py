#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Bool
import time
import math
from rclpy import qos

# ผมสร้าง Class PID ไว้ในนี้เลย เพื่อความชัวร์เรื่องเครื่องหมายครับ
class SimplePID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def calculate(self, error):
        self.integral += error
        # กันค่าสะสมบวมเกิน (Anti-windup)
        self.integral = max(min(self.integral, 50.0), -50.0)
        
        derivative = error - self.prev_error
        self.prev_error = error
        
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        return output

class AutoWalkVision(Node):
    def __init__(self):
        super().__init__('autowalk_vision')

        # --- Publishers ---
        self.send_robot_speed = self.create_publisher(
            Twist, "/galum/cmd_move/rpm", qos_profile=qos.qos_profile_system_default 
        )
        self.distance_pub = self.create_publisher(
            Float32, "/galum/current_distance", qos_profile=qos.qos_profile_system_default
        )
        
        # --- Subscribers ---
        # 1. เปลี่ยนจากรับ IMU มาเป็นรับ Vision Error
        self.create_subscription(Float32, "/galum/vision_error", self.vision_cb, 10)
        self.create_subscription(Bool, "/galum/is_found", self.found_cb, 10)
        
        # 2. Encoder (ใช้ Logic เดิมของพี่)
        self.create_subscription(
            Twist, "/galum/encoder", self.encoder_callback, qos_profile=qos.qos_profile_sensor_data
        )
        
        self.timer = self.create_timer(0.05, self.loop)

        # --- Movement Settings ---
        self.target_distance = 3.0  # เป้าหมาย 3 เมตร
        self.moveSpeed = 0.4        # ความเร็วพื้นฐาน (m/s) ลดลงนิดนึงเพื่อให้ Vision จับทัน
        
        self.vision_error = 0.0
        self.is_found = False
        
        self.motor1Speed = 0.0 # FL
        self.motor2Speed = 0.0 # FR
        self.motor3Speed = 0.0 # RL
        self.motor4Speed = 0.0 # RR
        
        self.stopped = False
        
        # --- Encoder Configuration (ตามที่พี่ส่งมา) ---
        self.wheel_radius = 0.05        
        self.ticks_per_rev = 7436       
        self.m_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        
        self.prev_ticks = [0, 0, 0, 0]
        self.total_distance = 0.0
        self.first_run = True   
        
        # --- Controller ---
        # ใช้ SimplePID ที่สร้างไว้ (Kp ควรเริ่มที่ค่าน้อยๆ เพราะ Error เป็น Pixel ไม่ใช่ Radian)
        self.controller = SimplePID(kp=0.002, ki=0.0, kd=0.0005)
        
        # [สำคัญ!] ตัวแปรกลับทิศทางพวงมาลัย
        # -1.0 = เพื่อแก้ให้เข้ากับสูตร v_left = speed + rotation
        self.steering_direction = -1.0 
        
        self.get_logger().info(f'VISION AUTO WALK START. Target: {self.target_distance}m')
    
    def vision_cb(self, msg):
        self.vision_error = msg.data

    def found_cb(self, msg):
        self.is_found = msg.data

    def encoder_callback(self, msg):
        # Logic การอ่าน Encoder เดิมของพี่ 100%
        current_ticks = [
            msg.linear.x, msg.linear.y, msg.angular.x, msg.angular.y
        ]
        
        if self.first_run:
            self.prev_ticks = current_ticks
            self.first_run = False
            return
        
        current_dist_step = 0.0
        
        for i in range(4):
            diff = current_ticks[i] - self.prev_ticks[i]
            
            # === POLARITY FIX (ตามของพี่) ===
            if i == 0 or i == 3:
                diff = -diff
            # ====================

            dist_m = diff * self.m_per_tick
            current_dist_step += dist_m
            
        avg_distance_step = current_dist_step / 4.0
        self.total_distance += avg_distance_step
        self.prev_ticks = current_ticks

        dist_msg = Float32()
        dist_msg.data = self.total_distance
        self.distance_pub.publish(dist_msg)

    def loop(self):
        if self.stopped:
            return

        if abs(self.total_distance) < self.target_distance:
            
            # 1. คำนวณ Rotation จาก Vision
            rotation = 0.0
            if self.is_found:
                raw_pid_output = self.controller.calculate(self.vision_error)
                # คูณ steering_direction (-1) เพื่อกลับทิศทางให้ถูกกับสูตรล้อ
                rotation = raw_pid_output * self.steering_direction
            else:
                rotation = 0.0 # ไม่เจอแปลง ให้เดินตรง
            
            # 2. คำนวณความเร็วล้อ (สูตรเดิมของพี่)
            # ถ้า rotation เป็นบวก -> v_left เพิ่ม -> เลี้ยวขวา
            v_left  = self.moveSpeed + rotation
            v_right = self.moveSpeed - rotation

            # 3. แปลง m/s เป็น RPM
            rpm_left  = (v_left  / (2 * math.pi * self.wheel_radius)) * 60.0
            rpm_right = (v_right / (2 * math.pi * self.wheel_radius)) * 60.0

            # 4. Map เข้าตัวแปรมอเตอร์ (ตามของพี่: 1,3=ซ้าย / 2,4=ขวา)
            self.motor1Speed = rpm_left
            self.motor3Speed = rpm_left
            self.motor2Speed = rpm_right
            self.motor4Speed = rpm_right
            
            self.sendData()
            
            status = "TRACKING" if self.is_found else "SEARCHING"
            print(f"[{status}] Dist: {self.total_distance:.2f}m | Err: {self.vision_error:.1f} | RPM: {int(rpm_left)}/{int(rpm_right)}", end='\r')
            
        else:
            # STOP
            self.motor1Speed  = 0.0; self.motor2Speed  = 0.0
            self.motor3Speed  = 0.0; self.motor4Speed  = 0.0
            self.sendData()
            
            if not self.stopped:
                print(f"\nSTOPPED! Final Distance: {self.total_distance:.4f} meters")
                self.stopped = True
        
    def sendData(self):
        motorspeed_msg = Twist()
        motorspeed_msg.linear.x = float(self.motor1Speed)
        motorspeed_msg.linear.y = float(self.motor2Speed)
        motorspeed_msg.angular.x = float(self.motor3Speed)
        motorspeed_msg.angular.y = float(self.motor4Speed)
        self.send_robot_speed.publish(motorspeed_msg)

def main():
    rclpy.init()
    node = AutoWalkVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()