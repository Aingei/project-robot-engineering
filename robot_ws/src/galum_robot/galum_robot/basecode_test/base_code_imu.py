#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32, String
from rclpy import qos
import math
import time

# ─────────────────────────────────────────────────────
#  Navigation States
# ─────────────────────────────────────────────────────
STATE_SEARCH              = 0
STATE_APPROACH            = 1
STATE_PAUSE               = 15
STATE_TURNL               = 3
STATE_FORWARD_AFTER_TURN  = 5
STATE_TURN_BACK_TO_ZERO   = 16  
STATE_FORWARD_3CM         = 17  # <--- State ใหม่สำหรับเดินหน้า 3 cm
STATE_SEARCH_WALL_ROTATE  = 8
STATE_ALIGN_WALL          = 4
STATE_PAUSE_AFTER_ALIGN   = 7
STATE_BACK_TO_TAG         = 6
STATE_BACK_BLIND          = 10  
STATE_RUN_SEQUENCE        = 11  
STATE_PAUSE_SEQUENCE      = 12  
STATE_PAUSE_BEFORE_SEQUENCE = 13 
STATE_TURNR                 = 14
STATE_WAIT                = 99

# ─────────────────────────────────────────────────────
#  Planting Mechanism States 
# ─────────────────────────────────────────────────────
STATE_SERVO1_FIRST        = 36
STATE_SERVO1_ON           = 34
STATE_SERVO1_OFF          = 35
STATE_PLANT_INIT          = 20
STATE_S4_ON               = 21
STATE_STP_DN1             = 22
STATE_S4_SWAP             = 23
STATE_STP_UP1             = 24
STATE_S4_OFF              = 25
STATE_MOVE_FWD            = 26
STATE_STP_DN2             = 27
STATE_S3_0                = 28
STATE_S3_100              = 29
STATE_STP_UP2             = 30
STATE_MOVE_BWD            = 31
STATE_S2_180              = 32
STATE_S2_0                = 33

class RobotMaster(Node):
    def __init__(self):
        super().__init__('robot_master')

        # ── Robot Geometry ──
        self.wheel_radius   = 0.06
        self.walk_speed     = 0.4
        self.align_speed    = 0.4 
        self.ticks_per_rev  = 7436
        self.m_per_tick     = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        
        self.path_mode = 0         

        # ── IMU / Relative Angle Tracking ──
        self.current_raw_yaw = 0.0    
        self.reference_yaw   = 0.0    
        self.relative_yaw    = 0.0    
        self.target_rel_yaw  = 0.0    
        self.kp_yaw          = 4.0    
        self.turn_timeout_start = 0.0
        
        # ─────────────────────────────────────────────────────
        #  Configuration: Turning Angles (Degrees)
        # ─────────────────────────────────────────────────────
        self.CFG_TURN_ANGLE_MODE1  = -55.0  # องศาที่เลี้ยวตอนเจอ Tag ใน Mode 1 (Right)
        self.CFG_TURN_ANGLE_MODE2  = -50.0  # องศาที่เลี้ยวตอนเจอ Tag ใน Mode 2 (Left)
        self.CFG_TURN_BACK_ANGLE   = 30.0   # องศาตอนตั้งลำกลับหลังเดินหน้าเสร็จ
        self.forward_after_turnr   = 0.40   # ระยะเดินหลังหมุนขวา
        
        # ── Ultrasonic Config ──
        self.US_TARGET_DIST = 0.04 
        self.FRONT_OFFSET   = -0.015
        self.REAR_OFFSET    = 0.08
        
        # ── PID GAINS ──
        self.KP_US_DIST  = 5.0      
        self.KP_US_ANGLE = 3.5     #4.5 
        self.KI_US       = 0.1            
        self.KD_US       = 0.8            
        
        self.align_start_time = 0.0
        self.integral_error   = 0.0
        self.prev_total_error = 0.0 
        
        self.pid_time_data = []
        self.pid_dist_data = []

        self.kp_tag  = 0.45
        self.state   = STATE_SERVO1_FIRST

        self.plant_dist_m     = 0.2
        self.plant_gap_m      = 0.1
        self.plant_interval_m = 0.2

        self.us_front = 0.0; self.us_rear = 0.0; self.us_valid = False
        self.us_front_buf = []; self.us_rear_buf = []
        self.US_FILTER_N = 5
        self.lost_wall_count = 0
        self.current_avg_dist = 0.0 
        
        self.tag_found = False; self.tag_err = 0.0; self.current_z_dist = 0.0
        self.back_tag_found = False; self.back_tag_y_ratio = 1.0; self.back_tag_threshold = 0.75
        self.total_dist = 0.0; self.prev_ticks = [0,0,0,0]; self.first_run = True
        self.cabbage_diameter = 0.0; self.logged_cabbage = False 
        
        # ── Timers ──
        self.pause_start_time = 0.0; self.forward_start_time = 0.0
        self.pause_align_start_time = 0.0; self.back_start_time = 0.0; self.blind_back_start_time = 0.0
        self.sequence_pause_start_time = 0.0; self.plant_timer = time.monotonic() 

        self.sequence_steps = []; self.current_step_index = 0
        self.s3_cycle_count = 0  # ตัวแปรนับรอบของ servo3

        # ── Publishers ──
        self.cmd_pub = self.create_publisher(Twist, "/galum/cmd_move/rpm", 10)
        self.scan_mode_pub = self.create_publisher(Float32, "/galum/scan_mode", 10)
        self.servo_pub = self.create_publisher(Twist, '/galum/servo/angle', 10)
        self.stepper_pub = self.create_publisher(Twist, '/galum/stepper/angle', 10)
        self.save_img_pub = self.create_publisher(String, "/galum/save_image_cmd", 10)
        self.dist_pub = self.create_publisher(Float32, "/galum/current_dist", 10)
        self.pid_plot_pub = self.create_publisher(String, "/galum/pid_plot_data", 10)

        # ── Subscribers ──
        self.create_subscription(Float32MultiArray, "/galum/us_dual", self.us_cb, qos.qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, "/galum/vision_packet", self.vision_cb, qos.qos_profile_sensor_data)
        self.create_subscription(Twist, "/galum/encoder", self.encoder_cb, qos.qos_profile_sensor_data)
        self.create_subscription(Float32, "/galum/imu/yaw", self.imu_cb, qos.qos_profile_sensor_data)
        
        self.current_servo1 = 90.0; self.target_servo1 = 90.0   
        self.current_servo2 = 180.0; self.current_servo3 = 180.0; self.current_servo4 = 90.0

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("✅ Robot Master: IMU Relative Angle Feedback Enabled")

    # ─────────────────────────────────────────────────────
    #  Helper Methods
    # ─────────────────────────────────────────────────────
    def imu_cb(self, msg):
        self.current_raw_yaw = msg.data

    def get_diff(self, target, current):
        diff = target - current
        while diff > 180: diff -= 360
        while diff < -180: diff += 360
        return diff
    
    def _reset_align_state(self):
        self.total_dist = 0.0; self.integral_error = 0.0; self.prev_total_error = 0.0
        self.align_start_time = time.monotonic()
        # 📌 ล้างข้อมูลเก่า
        self.pid_time_data.clear()
        self.pid_dist_data.clear()

    def us_cb(self, msg):
        if len(msg.data) >= 2:
            f, r = msg.data[0], msg.data[1]
            if 0.005 < f < 2.5 and 0.005 < r < 2.5:
                self.us_front_buf.append(f); self.us_rear_buf.append(r)
                if len(self.us_front_buf) > self.US_FILTER_N:
                    self.us_front_buf.pop(0); self.us_rear_buf.pop(0)
                self.us_front = sum(self.us_front_buf) / len(self.us_front_buf)
                self.us_rear  = sum(self.us_rear_buf)  / len(self.us_rear_buf)
                self.us_valid = True; self.lost_wall_count = 0
                self.current_avg_dist = (self.us_front + (self.us_rear + self.REAR_OFFSET)) / 2.0
            else:
                self.lost_wall_count += 1
                if self.lost_wall_count > 5: self.us_valid = False

    def vision_cb(self, msg):
        d = msg.data; self.tag_found = (d[0] == 1.0); self.tag_err = d[1]
        if len(d) > 6:
            self.plant_dist_m, self.plant_gap_m, self.plant_interval_m = d[4]/100.0, d[5]/100.0, d[6]/100.0
        self.current_z_dist = d[7] if len(d) > 7 else 0.0
        self.back_tag_found = (d[12] == 1.0); self.back_tag_y_ratio = d[13] if len(d) > 13 else 1.0
        self.cabbage_diameter = d[14] if len(d) > 14 else 0.0

    def encoder_cb(self, msg):
        curr = [msg.linear.x, msg.linear.y, msg.angular.x, msg.angular.y]
        if self.first_run: self.prev_ticks = curr; self.first_run = False; return
        d = 0.0
        for i in range(4):
            diff = curr[i] - self.prev_ticks[i]
            if i == 0 or i == 3: diff = -diff
            d += diff * self.m_per_tick
        self.total_dist += (d / 4.0); self.prev_ticks = curr

    def _reset_align_state(self):
        self.total_dist = 0.0; self.integral_error = 0.0; self.prev_total_error = 0.0
        self.align_start_time = time.monotonic()

    def _calculate_dual_pid(self):
        if not self.us_valid: return 0.0, 0.0
        avg_dist = self.current_avg_dist; err_dist = avg_dist - self.US_TARGET_DIST 
        real_front = self.us_front; real_rear = self.us_rear + self.REAR_OFFSET
        diff_angle = real_front - real_rear
        total_error = (err_dist * self.KP_US_DIST) + (diff_angle * self.KP_US_ANGLE)
        self.integral_error += total_error; self.integral_error = max(-0.1, min(0.1, self.integral_error)) 
        derivative = total_error - self.prev_total_error; self.prev_total_error = total_error 
        pid_out = total_error + (self.integral_error * self.KI_US) + (derivative * self.KD_US)
        max_turn = self.align_speed * 0.8
        return max(-max_turn, min(max_turn, pid_out)), avg_dist
    
    def _pid_to_rpm(self, pid_out, base_speed):
        v_l = base_speed + pid_out; v_r = base_speed - pid_out
        return self._m_s_to_rpm(v_l), self._m_s_to_rpm(v_r)

    def _get_reverse_rpm(self, base_speed):
        if not self.us_valid: return self._m_s_to_rpm(base_speed), self._m_s_to_rpm(base_speed)
        real_front = self.us_front + self.FRONT_OFFSET; real_rear = self.us_rear
        diff_angle = real_front - real_rear; err_dist = self.current_avg_dist - self.US_TARGET_DIST
        pid_out = (diff_angle * 6.0) - (err_dist * 1.5)
        max_turn = self.align_speed * 0.6; pid_out = max(-max_turn, min(max_turn, pid_out))
        return self._m_s_to_rpm(base_speed + pid_out), self._m_s_to_rpm(base_speed - pid_out)

    def _m_s_to_rpm(self, v): return (v / (2 * math.pi * self.wheel_radius)) * 60
    def _set_scan_mode(self, mode: float): msg = Float32(); msg.data = mode; self.scan_mode_pub.publish(msg)
    def send_rpm(self, l, r): 
        msg = Twist(); msg.linear.x = float(l); msg.angular.x = float(l); msg.linear.y = float(r); msg.angular.y = float(r); self.cmd_pub.publish(msg)
    def send_stepper_cmd(self, direction, pulses):
        msg = Twist(); msg.linear.x = float(direction); msg.linear.y = float(pulses); self.stepper_pub.publish(msg)
    def send_servo_cmd(self):
        msg = Twist(); msg.linear.x = float(self.current_servo1); msg.linear.y = float(self.current_servo2); msg.linear.z = float(self.current_servo3); msg.angular.x = float(self.current_servo4); self.servo_pub.publish(msg)

    def _send_pid_data_to_plot(self):
        if len(self.pid_time_data) == 0: return
        
        # แปลง List ให้เป็น String ยาวๆ เพื่อส่งผ่าน Topic
        # รูปแบบ: "time1,time2,...|dist1,dist2,...|target_dist"
        time_str = ",".join(f"{t:.2f}" for t in self.pid_time_data)
        dist_str = ",".join(f"{d:.4f}" for d in self.pid_dist_data)
        payload = f"{time_str}|{dist_str}|{self.US_TARGET_DIST}"
        
        msg = String()
        msg.data = payload
        self.pid_plot_pub.publish(msg)
        self.get_logger().info(" สั่งพล็อตกราฟ PID ผ่าน Topic /galum/pid_plot_data แล้ว!")
        
    # ─────────────────────────────────────────────────────
    #  Main Control Loop
    # ─────────────────────────────────────────────────────
    def control_loop(self):
        left_rpm = 0.0; right_rpm = 0.0

        if self.state == STATE_SERVO1_FIRST:
            if time.monotonic() - self.plant_timer > 1.0: 
                self.target_servo1 = 180.0; self.state = STATE_SEARCH 
                
        elif self.state == STATE_SEARCH:
            if self.tag_found: 
                if self.path_mode == 0:
                    self.path_mode = 1 if self.tag_err < 0 else 2
                    self.get_logger().info(f"📍 Mode Locked: {self.path_mode}")
                self.state = STATE_APPROACH
            else: 
                rpm = self._m_s_to_rpm(self.walk_speed); left_rpm, right_rpm = rpm, rpm

        elif self.state == STATE_APPROACH:
            stop_dist = 0.45 if self.path_mode == 1 else 0.6
            if self.tag_found and (0.0 < self.current_z_dist < stop_dist):
                self.pause_start_time = time.monotonic(); self.state = STATE_PAUSE; self.send_rpm(0, 0); return
            elif not self.tag_found: self.state = STATE_SEARCH
            else: pid = self.tag_err * self.kp_tag; left_rpm, right_rpm = 60.0 - pid, 60.0 + pid

        elif self.state == STATE_PAUSE:
            if time.monotonic() - self.pause_start_time > 2.0:
                self.reference_yaw = self.current_raw_yaw
                #self.target_rel_yaw = -50.0 if self.path_mode == 1 else -40.0
                self.target_rel_yaw = self.CFG_TURN_ANGLE_MODE1 if self.path_mode == 1 else self.CFG_TURN_ANGLE_MODE2
                self.turn_timeout_start = time.monotonic()
                self.state = STATE_TURNL
                self.get_logger().info(f"🔄 Turn Start! From: {self.reference_yaw:.1f}° Target: +{self.target_rel_yaw}°")

        elif self.state == STATE_TURNL:
            self.relative_yaw = self.get_diff(self.current_raw_yaw, self.reference_yaw)
            yaw_error = self.get_diff(self.target_rel_yaw, self.relative_yaw)

            self.get_logger().info(f"Turning... Current: {self.relative_yaw:.2f}° | Desire: {self.target_rel_yaw:.2f}°")

            elapsed = time.monotonic() - self.turn_timeout_start
            if abs(yaw_error) < 3.0 or elapsed > 12.0:
                if elapsed > 12.0: self.get_logger().warn("⚠️ Turn Timeout reached!")
                self.get_logger().info("✅ Turn Complete!")
                self.send_rpm(0, 0); self.forward_start_time = time.monotonic(); self.total_dist = 0.0;self.state = STATE_FORWARD_AFTER_TURN
            else:   
                speed = yaw_error * self.kp_yaw
                speed = max(-80.0, min(80.0, speed))
                left_rpm, right_rpm = speed, -speed
        
        elif self.state == STATE_FORWARD_AFTER_TURN:
            target_distance = 0.45 if self.path_mode == 1 else 0.4
            if abs(self.total_dist) < target_distance:
                rpm = self._m_s_to_rpm(self.walk_speed)
                left_rpm, right_rpm = rpm, rpm
            else: 
                self.send_rpm(0, 0) # เบรกรถก่อน
                
                # 📌 เช็คว่าเป็น Mode 1 หรือ Mode 2
                if self.path_mode == 1:
                    # Mode 1: สั่งหมุนตั้งลำตามปกติ
                    self.target_rel_yaw = self.CFG_TURN_BACK_ANGLE 
                    self.turn_timeout_start = time.monotonic()
                    self.state = STATE_TURN_BACK_TO_ZERO
                    self.get_logger().info(f"🔄 Walked {abs(self.total_dist):.2f}m. Returning heading (Mode 1)...")
                else:
                    # Mode 2: ไม่หมุนกลับ! ข้ามไปหันหน้าเข้าหากำแพงเลย
                    self.get_logger().info(f"🔄 Walked {abs(self.total_dist):.2f}m. Skipping Turn Back (Mode 2)...")
                    self.state = STATE_SEARCH_WALL_ROTATE

        # ── State หมุนกลับมาที่ 0 องศา (ตั้งลำให้ตรงก่อนเดินหน้า) ──
        elif self.state == STATE_TURN_BACK_TO_ZERO:
            self.relative_yaw = self.get_diff(self.current_raw_yaw, self.reference_yaw)
            yaw_error = self.get_diff(self.target_rel_yaw, self.relative_yaw)

            self.get_logger().info(f"Turning Back... Current: {self.relative_yaw:.2f}° | Desire: {self.target_rel_yaw:.2f}°")

            elapsed = time.monotonic() - self.turn_timeout_start
            if abs(yaw_error) < 3.0 or elapsed > 12.0:
                if elapsed > 12.0: self.get_logger().warn("⚠️ Turn Back Timeout reached!")
                self.get_logger().info("✅ Turn Back Complete! Ready to move forward 3cm.")
                self.send_rpm(0, 0)
                
                # 📌 รีเซ็ตระยะทางก่อนเริ่มเดินหน้า 3 cm
                self.total_dist = 0.0  
                self.state = STATE_FORWARD_3CM
            else:   
                speed = yaw_error * self.kp_yaw
                speed = max(-80.0, min(80.0, speed))
                left_rpm, right_rpm = speed, -speed

        # ── State ใหม่: เดินหน้า 3 cm ──
        elif self.state == STATE_FORWARD_3CM:
            target_distance = self.forward_after_turnr  # 3 เซนติเมตร
            
            if abs(self.total_dist) < target_distance:
                # ใช้ความเร็วลดลงครึ่งนึง (0.5) เพื่อให้วิ่งระยะสั้นได้แม่นยำ ไม่เลยเป้า
                rpm = self._m_s_to_rpm(self.walk_speed * 0.5) 
                left_rpm, right_rpm = rpm, rpm
            else:
                self.send_rpm(0, 0)
                self.get_logger().info(f"✅ Moved forward {abs(self.total_dist):.2f}m. Ready to align wall.")
                self.state = STATE_SEARCH_WALL_ROTATE

        elif self.state == STATE_SEARCH_WALL_ROTATE:
            if self.us_valid: self._reset_align_state(); self.state = STATE_ALIGN_WALL
            else: left_rpm, right_rpm = 60.0, -60.0 


        elif self.state == STATE_ALIGN_WALL:
            if time.monotonic() - self.align_start_time > 20.0:
                # 📌 สั่งเซฟกราฟกรณีหมดเวลา 20 วิ
                self._send_pid_data_to_plot() 
                self.pause_align_start_time = time.monotonic(); self.state = STATE_PAUSE_AFTER_ALIGN
            elif not self.us_valid: self.state = STATE_SEARCH_WALL_ROTATE
            else:
                pid_out, avg_dist = self._calculate_dual_pid()
                left_rpm, right_rpm = self._pid_to_rpm(pid_out, self.align_speed)
                is_parallel = abs((self.us_front + self.FRONT_OFFSET) - self.us_rear) < 0.03
                
                self.pid_time_data.append(time.monotonic() - self.align_start_time)
                self.pid_dist_data.append(avg_dist)
                
                if abs(self.total_dist) >= 0.20 and is_parallel and abs(avg_dist - self.US_TARGET_DIST) < 0.05:
                    # 📌 สั่งเซฟกราฟกรณีตีคู่กำแพงสำเร็จ
                    self._send_pid_data_to_plot() 
                    self.pause_align_start_time = time.monotonic(); self.state = STATE_PAUSE_AFTER_ALIGN
                    
        elif self.state == STATE_PAUSE_AFTER_ALIGN:
            if time.monotonic() - self.pause_align_start_time > 2.0:
                self._set_scan_mode(1.0); self.back_start_time = time.monotonic(); self.state = STATE_BACK_TO_TAG

        elif self.state == STATE_BACK_TO_TAG:
            back_speed = -self.walk_speed * 0.5
            
            # สั่งให้ถอยหลังตีคู่กำแพงตลอดเวลาที่อยู่ใน State นี้
            left_rpm, right_rpm = self._get_reverse_rpm(back_speed)

            if time.monotonic() - self.back_start_time > 2.0:
                # 📌 กำหนดตำแหน่งแกน Y ที่ต้องการให้หยุด (0.0=บนสุด, 1.0=ขอบล่างสุด)
                # จากรูปที่ 2 Tag เลื่อนลงมาค่อนข้างต่ำ ลองตั้งเป้าไว้ที่ 0.80 ก่อนครับ
                TARGET_Y_RATIO = 0.80  
                
                if self.back_tag_found:
                    # แปะ Flag ไว้ว่า "เราเคยเห็น Tag แล้วนะ"
                    self.has_seen_back_tag = True 
                    
                    # ถ้า Tag เลื่อนลงมาถึงตำแหน่งในรูปที่ 2 แล้ว ให้หยุด!
                    if self.back_tag_y_ratio >= TARGET_Y_RATIO:
                        self.get_logger().info(f"🎯 Reached Tag Target Position! (Y: {self.back_tag_y_ratio:.2f})")
                        self.send_rpm(0, 0)
                        self.sequence_steps = [self.plant_dist_m, self.plant_dist_m, self.plant_gap_m, self.plant_interval_m, self.plant_interval_m, self.plant_interval_m]
                        self._set_scan_mode(0.0)
                        self.total_dist = 0.0
                        self.current_step_index = 0
                        self.pause_start_time = time.monotonic()
                        
                        # 🚀 ข้ามโหมดถอยตาบอด ไปเข้า Sequence ปลูกเลย
                        self.state = STATE_PAUSE_BEFORE_SEQUENCE
                        
                elif getattr(self, 'has_seen_back_tag', False):
                    # 🛡️ กันเหนียว: ถ้าเคยเห็น Tag แล้ว (รูป 1) แต่อยู่ๆ มันหลุดขอบจอด้านล่างไป (มองไม่เห็นแล้ว)
                    # แสดงว่าถอยมาลึกเกินไปแล้ว ให้รีบเบรกแล้วเริ่มปลูกเลย
                    self.get_logger().info("⚠️ Tag dropped out of view! Stopping just in case.")
                    self.send_rpm(0, 0)
                    self.sequence_steps = [self.plant_dist_m, self.plant_dist_m, self.plant_gap_m, self.plant_interval_m, self.plant_interval_m, self.plant_interval_m]
                    self._set_scan_mode(0.0)
                    self.total_dist = 0.0
                    self.current_step_index = 0
                    self.pause_start_time = time.monotonic()
                    self.state = STATE_PAUSE_BEFORE_SEQUENCE

        # 🗑️ หมายเหตุ: ไม่ต้องมี elif self.state == STATE_BACK_BLIND: แล้วนะครับ ลบทิ้งได้เลย
        
        elif self.state == STATE_PAUSE_BEFORE_SEQUENCE:
            if time.monotonic() - self.pause_start_time > 2.0:
                if self.current_step_index >= 3: self._set_scan_mode(2.0)
                self.state = STATE_RUN_SEQUENCE

        elif self.state == STATE_RUN_SEQUENCE:
            if self.current_step_index >= len(self.sequence_steps):
                self._set_scan_mode(0.0); self.state = STATE_WAIT
            elif abs(self.total_dist) >= self.sequence_steps[self.current_step_index]:
                self.send_rpm(0, 0); self.logged_cabbage = False 
                if self.current_step_index < 2:
                    self.target_servo1 = 90.0; self.plant_timer = time.monotonic(); self.state = STATE_SERVO1_ON  
                else: self.sequence_pause_start_time = time.monotonic(); self.state = STATE_PAUSE_SEQUENCE
            else:
                rpm = self._m_s_to_rpm(self.walk_speed); left_rpm, right_rpm = rpm, rpm

        # --- Planting Logic ---
        elif self.state == STATE_SERVO1_ON:
            if time.monotonic() - self.plant_timer > 2.0: self.plant_timer = time.monotonic(); self.state = STATE_PLANT_INIT
            
        elif self.state == STATE_PLANT_INIT:
            if time.monotonic() - self.plant_timer > 1.0: self.current_servo4 = 60.0; self.plant_timer = time.monotonic(); self.state = STATE_S4_ON
            
        elif self.state == STATE_S4_ON:
            if time.monotonic() - self.plant_timer > 2.0: self.send_stepper_cmd(-1.0, 350); self.plant_timer = time.monotonic(); self.state = STATE_STP_DN1
            
        elif self.state == STATE_STP_DN1:
            if time.monotonic() - self.plant_timer > 3.0: self.current_servo4 = 120.0; self.plant_timer = time.monotonic(); self.state = STATE_S4_SWAP
            
        elif self.state == STATE_S4_SWAP:
            if time.monotonic() - self.plant_timer > 2.0: self.send_stepper_cmd(1.0, 350); self.plant_timer = time.monotonic(); self.state = STATE_STP_UP1
            
        elif self.state == STATE_STP_UP1:
            if time.monotonic() - self.plant_timer > 4.0: self.current_servo4 = 90.0; self.plant_timer = time.monotonic(); self.state = STATE_S4_OFF
            
        elif self.state == STATE_S4_OFF:
            if time.monotonic() - self.plant_timer > 2.0: self.total_dist = 0.0; self.plant_timer = time.monotonic(); self.state = STATE_MOVE_FWD
            
        # ── เดินหน้า 4.5 cm (0.045 m) ──
        elif self.state == STATE_MOVE_FWD:
            if abs(self.total_dist) < 0.045: rpm = self._m_s_to_rpm(self.walk_speed * 0.5); left_rpm, right_rpm = rpm, rpm
            else: self.send_rpm(0, 0); self.send_stepper_cmd(-1.0, 150); self.plant_timer = time.monotonic(); self.state = STATE_STP_DN2
            
        # ── Servo3 3 Cycles Logic ──
        elif self.state == STATE_STP_DN2:
            if time.monotonic() - self.plant_timer > 2.0: 
                self.s3_cycle_count = 1
                self.current_servo3 = 0.0  # เปิดครั้งที่ 1
                self.plant_timer = time.monotonic()
                self.state = STATE_S3_100

        elif self.state == STATE_S3_100:
            if time.monotonic() - self.plant_timer > 1.0: # รอเวลาให้ servo เปิดสุด (1 วิ)
                if self.s3_cycle_count < 3:
                    self.current_servo3 = 180.0  # สั่งปิด
                    self.plant_timer = time.monotonic()
                    self.state = STATE_S3_0
                else:
                    # ถ้าเปิดครั้งที่ 3 เสร็จแล้ว -> สั่ง Stepper ขึ้น
                    self.send_stepper_cmd(1.0, 800)
                    self.plant_timer = time.monotonic()
                    self.state = STATE_S3_0
                    self.s3_cycle_count = 99  # เซ็ต Flag ว่ากำลังรอ Stepper ขึ้น

        elif self.state == STATE_S3_0:
            # ถ้าเป็นจังหวะรอ Stepper (99) ให้รอ 2 วินาที, ถ้าแค่รอปิดพับให้รอ 1 วินาที
            wait_time = 2.0 if self.s3_cycle_count == 99 else 1.0 
            
            if time.monotonic() - self.plant_timer > wait_time:
                if self.s3_cycle_count == 99:
                    # Stepper ขึ้นเสร็จแล้ว -> ปิด Servo จบงาน
                    self.current_servo3 = 180.0 
                    self.plant_timer = time.monotonic()
                    self.state = STATE_STP_UP2
                else:
                    # ปิดเสร็จแล้ว -> นับรอบเพิ่มและสั่งเปิดรอบถัดไป
                    self.s3_cycle_count += 1
                    self.current_servo3 = 0.0  # เปิดครั้งที่ 2 และ 3
                    self.plant_timer = time.monotonic()
                    self.state = STATE_S3_100
    
        elif self.state == STATE_STP_UP2:
            if time.monotonic() - self.plant_timer > 4.0: self.total_dist = 0.0; self.plant_timer = time.monotonic(); self.state = STATE_MOVE_BWD
            
        # ── ถอยหลัง 4.5 cm (0.045 m) ──
        elif self.state == STATE_MOVE_BWD:
            if abs(self.total_dist) < 0.045: rpm = self._m_s_to_rpm(-self.walk_speed * 0.5); left_rpm, right_rpm = rpm, rpm
            else: self.send_rpm(0, 0); self.current_servo2 = 90.0; self.plant_timer = time.monotonic(); self.state = STATE_S2_180
            
        elif self.state == STATE_S2_180:
            if time.monotonic() - self.plant_timer > 2.0: self.current_servo2 = 180.0; self.plant_timer = time.monotonic(); self.state = STATE_S2_0
            
        elif self.state == STATE_S2_0:
            if time.monotonic() - self.plant_timer > 2.0: self.target_servo1 = 90.0; self.plant_timer = time.monotonic(); self.state = STATE_SERVO1_OFF 
            
        elif self.state == STATE_SERVO1_OFF:
            if time.monotonic() - self.plant_timer > 2.0: self.target_servo1 = 180.0; self.sequence_pause_start_time = time.monotonic(); self.state = STATE_PAUSE_SEQUENCE 

        elif self.state == STATE_PAUSE_SEQUENCE:
            elapsed = time.monotonic() - self.sequence_pause_start_time
            if self.current_step_index >= 3 and 1.0 < elapsed < 2.0 and not self.logged_cabbage:
                self.get_logger().info(f"🟢 Cabbage {self.current_step_index-2}: {self.cabbage_diameter:.2f}cm")
                self.logged_cabbage = True; msg = String(); msg.data = f"cabbage_{self.current_step_index-2}.jpg"; self.save_img_pub.publish(msg)
            if elapsed > 2.0:
                self.total_dist = 0.0; self.current_step_index += 1
                self._set_scan_mode(2.0 if self.current_step_index >= 3 else 0.0); self.state = STATE_RUN_SEQUENCE 

        # ── Final Execution ──
        self.send_rpm(left_rpm, right_rpm)
        if self.current_servo1 < self.target_servo1: self.current_servo1 = min(self.current_servo1 + 1.0, self.target_servo1)
        elif self.current_servo1 > self.target_servo1: self.current_servo1 = max(self.current_servo1 - 1.0, self.target_servo1)
        self.send_servo_cmd()
        self.dist_pub.publish(Float32(data=float(self.total_dist)))

def main():
    rclpy.init(); node = RobotMaster()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()