#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math
from rclpy import qos

# กำหนดสถานะการทำงาน
STATE_WALK = 0       # เดินหน้า
STATE_WAIT = 1       # หยุดพักแป๊บนึง
STATE_TURN_RIGHT = 2 # หมุนขวา
STATE_END = 3        # จบการทำงาน

class WalkStopTurnRightEnd(Node):
    def __init__(self):
        super().__init__('walk_stop_turn_end')

        self.send_robot_speed = self.create_publisher(
            Twist, "/galum/cmd_move/rpm", qos_profile=qos.qos_profile_system_default)
        
        self.create_subscription(Twist, "/galum/encoder", self.encoder_callback, qos_profile=qos.qos_profile_sensor_data)
        
        self.timer = self.create_timer(0.05, self.loop)

        # ===== ตั้งค่าพารามิเตอร์ =====
        self.target_dist = 1.0        # เดิน 1 เมตร
        self.target_angle = 90.0      # หมุนขวา 90 องศา (แก้เป็น 90 ให้เห็นชัดๆ หรือปรับตามต้องการ)
        
        self.walk_rpm = 100.0         # ความเร็วเดิน
        self.turn_rpm = 50.0          # ความเร็วหมุน
        
        # --- Config ตาม AutoWalk ---
        self.wheel_radius = 0.05      # รัศมีล้อ 0.05m
        self.ticks_per_rev = 7436     # *** แก้เป็นค่าที่ถูกต้องของฮาร์ดแวร์คุณ ***
        
        # ระยะห่างล้อ (Track Width) สำคัญมากสำหรับการเลี้ยว
        # ถ้าเลี้ยวไม่ครบองศา ให้ปรับค่านี้ (ถ้ารถหมุนน้อยไป -> เพิ่มค่านี้ / หมุนเกิน -> ลดค่านี้)
        self.track_width = 0.20       
        # ==========================

        self.state = STATE_WALK
        
        # ตัวแปรคำนวณระยะทาง
        self.m_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        
        self.prev_ticks = [0,0,0,0]
        self.dist_accum = 0.0         # ตัวสะสมระยะทาง
        self.first_run = True
        
        self.wait_start_time = 0.0

        self.get_logger().info('START: Walk -> Stop -> Turn Right -> END')

    def encoder_callback(self, msg):
        # รับค่าจาก Encoders
        current_ticks = [msg.linear.x, msg.linear.y, msg.angular.x, msg.angular.y]
        
        if self.first_run:
            self.prev_ticks = current_ticks
            self.first_run = False
            return

        total_step_dist = 0.0
        
        for i in range(4):
            diff = current_ticks[i] - self.prev_ticks[i]
            
            # === POLARITY FIX (แบบ AutoWalk) ===
            # กลับค่าล้อ FL(0) และ RR(3) ให้เป็นทิศทางที่ถูกต้อง
            if i == 0 or i == 3:
                diff = -diff
            # ===================================
            
            # แปลงเป็นเมตร
            dist_m = diff * self.m_per_tick
            
            # ใช้ abs() เพื่อสะสมระยะทางที่ล้อหมุนจริง 
            # (ใช้ได้ทั้งตอนเดินหน้า และตอนหมุนเลี้ยว)
            total_step_dist += abs(dist_m)

        # ระยะเฉลี่ยที่ล้อทั้ง 4 ขยับไป
        avg_dist = total_step_dist / 4.0
        
        self.dist_accum += avg_dist
        self.prev_ticks = current_ticks

    def loop(self):
        
        # 1. เดินหน้า
        if self.state == STATE_WALK:
            if self.dist_accum < self.target_dist:
                # เดินหน้า: ล้อซ้าย(+), ล้อขวา(+)
                self.set_speed(self.walk_rpm, self.walk_rpm) 
                # Print ดูระยะทาง
                print(f"Walking: {self.dist_accum:.3f} / {self.target_dist} m", end='\r')
            else:
                # ครบระยะ -> หยุด -> ไปสถานะพัก
                self.set_speed(0, 0)
                self.dist_accum = 0.0  # รีเซ็ตระยะทางเพื่อนับใหม่ตอนเลี้ยว
                self.wait_start_time = time.time()
                self.state = STATE_WAIT
                print(f"\nWalk Done. Waiting...")

        # 2. หยุดพัก (Wait)
        elif self.state == STATE_WAIT:
            # หยุดนิ่งๆ 1 วินาที
            if time.time() - self.wait_start_time < 1.0:
                self.set_speed(0, 0)
            else:
                self.state = STATE_TURN_RIGHT
                print("Start Turning Right...")

        # 3. หมุนขวา (Turn Right)
        elif self.state == STATE_TURN_RIGHT:
            # คำนวณระยะทางที่ล้อต้องวิ่งเพื่อให้ตัวรถหมุนตามองศา
            # Arc Length = Radius * Angle(radians)
            # Radius ในที่นี้คือ ครึ่งหนึ่งของความกว้างฐานล้อ (track_width / 2)
            target_wheel_dist = (self.track_width / 2.0) * math.radians(self.target_angle)
            
            if self.dist_accum < target_wheel_dist:
                # หมุนขวา: ล้อซ้ายเดินหน้า(+), ล้อขวาถอยหลัง(-)
                self.set_speed(self.turn_rpm, -self.turn_rpm)
                print(f"Turning: {self.dist_accum:.3f} / {target_wheel_dist:.3f} m (Arc)", end='\r')
            else:
                # ครบองศา -> จบงาน
                self.set_speed(0, 0)
                self.state = STATE_END
                print(f"\nTurn Done. Mission Complete.")

        # 4. จบ (End)
        elif self.state == STATE_END:
            self.set_speed(0, 0)

    def set_speed(self, left_rpm, right_rpm):
        msg = Twist()
        # ส่งค่า RPM ตรงๆ (Python แปลง, C++ รับไปขับมอเตอร์)
        msg.linear.x = float(left_rpm)   # FL
        msg.angular.x = float(left_rpm)  # RL
        msg.linear.y = float(right_rpm)  # FR
        msg.angular.y = float(right_rpm) # RR
        self.send_robot_speed.publish(msg)

def main():
    rclpy.init()
    node = WalkStopTurnRightEnd()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.set_speed(0,0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()