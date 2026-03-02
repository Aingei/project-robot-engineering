#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32
from rclpy import qos
import math
import time

# ─────────────────────────────────────────────────────
#  States
# ─────────────────────────────────────────────────────
STATE_SEARCH              = 0
STATE_APPROACH            = 1
STATE_PAUSE               = 15
STATE_TURNL               = 3
STATE_FORWARD_SCAN        = 5
STATE_ALIGN_WALL          = 4
STATE_PAUSE_AFTER_ALIGN   = 7
STATE_BACK_TO_TAG         = 6
STATE_GOTO_FIRST          = 2
STATE_WAIT                = 99

class RobotMaster(Node):
    def __init__(self):
        super().__init__('robot_master')

        # ── Robot Geometry ──────────────────────────────────────
        self.wheel_radius   = 0.05
        self.walk_speed     = 0.4
        self.ticks_per_rev  = 7436
        self.m_per_tick     = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev

        # ── Thresholds ──────────────────────────────────────────
        self.stop_distance_threshold = 0.5  # หยุดหน้า Tag
        
        # ── Dual Ultrasonic Config ──────────────────────────────
        # Target: ให้ห่างจากแปลง 35 cm
        self.US_TARGET_DIST = 0.35 
        
        # Kp_dist: คุมระยะห่าง (Error เป็นเมตร)
        self.KP_US_DIST  = 4.0   
        
        # Kp_angle: คุมความขนาน (Front - Rear)
        # ถ้า Front < Rear (ค่าติดลบ) = หัวปักเข้า -> ต้องเลี้ยวขวา (PID เป็นบวก)
        # สูตรคือ: Angle_Term * -1.0
        self.KP_US_ANGLE = 6.0   

        # ── Vision PID (Tag Only) ───────────────────────────────
        self.kp_tag  = 0.45

        # ── Variables ───────────────────────────────────────────
        self.state    = STATE_SEARCH
        self.target_m = 0.5  # ระยะที่จะวิ่งยาว (รับจาก Tag)

        # Ultrasonic Data
        self.us_front = 0.0
        self.us_rear  = 0.0
        self.us_valid = False

        # Tag Data (AprilTag)
        self.tag_found = False
        self.tag_err   = 0.0
        self.current_z_dist = 0.0
        
        # Back Tag Data (สำหรับถอยหลัง)
        self.back_tag_found   = False
        self.back_tag_y_ratio = 1.0
        self.back_tag_threshold = 0.25 # Tag อยู่ขอบบน 25% ให้หยุด

        # Encoder & Timers
        self.total_dist = 0.0
        self.prev_ticks = [0, 0, 0, 0]
        self.first_run  = True

        self.pause_start_time       = 0.0
        self.turn_start_time        = 0.0
        self.forward_start_time     = 0.0
        self.pause_align_start_time = 0.0

        # ── ROS I/O ─────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, "/galum/cmd_move/rpm", 10)
        self.scan_mode_pub = self.create_publisher(Float32, "/galum/scan_mode", 10)

        # รับค่า Ultrasonic 2 ตัว [Front, Rear]
        self.create_subscription(Float32MultiArray, "/galum/us_dual",
                                 self.us_cb, qos.qos_profile_sensor_data)
        
        # รับค่า Vision (AprilTag)
        self.create_subscription(Float32MultiArray, "/galum/vision_packet",
                                 self.vision_cb, qos.qos_profile_sensor_data)
        
        self.create_subscription(Twist, "/galum/encoder",
                                 self.encoder_cb, qos.qos_profile_sensor_data)

        self.debug_pid_pub = self.create_publisher(Float32, "/debug/pid_output", 10)

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Robot Master: Dual Ultrasonic Mode (YOLO Removed)")

    # ─────────────────────────────────────────────────────
    #  Callbacks
    # ─────────────────────────────────────────────────────
    def us_cb(self, msg):
        """รับค่า Ultrasonic [Front, Rear]"""
        if len(msg.data) >= 2:
            f = msg.data[0]
            r = msg.data[1]
            # กรองค่ารบกวน (รับเฉพาะ 5cm - 2m)
            if 0.05 < f < 2.0 and 0.05 < r < 2.0:
                self.us_front = f
                self.us_rear  = r
                self.us_valid = True
            else:
                self.us_valid = False

    def vision_cb(self, msg):
        """รับค่า Tag อย่างเดียว (ไม่สน YOLO)"""
        d = msg.data
        self.tag_found  = (d[0] == 1.0)
        self.tag_err    = d[1]
        
        # ถ้าเจอ Tag และอยู่ในโหมด SEARCH ให้รับค่าระยะวิ่ง (Target Dist)
        if self.tag_found and len(d) > 4 and d[4] > 0 and self.state == STATE_SEARCH:
            self.target_m = d[4] / 100.0  # แปลง cm -> m

        self.current_z_dist = d[7]  if len(d) > 7  else 0.0
        
        # รับค่า Back Scan
        self.back_tag_found   = (d[12] == 1.0) if len(d) > 12 else False
        self.back_tag_y_ratio = d[13]           if len(d) > 13 else 1.0

    def encoder_cb(self, msg):
        curr = [msg.linear.x, msg.linear.y, msg.angular.x, msg.angular.y]
        if self.first_run:
            self.prev_ticks = curr; self.first_run = False; return
        d = 0.0
        for i in range(4):
            diff = curr[i] - self.prev_ticks[i]
            if i == 0 or i == 3: diff = -diff
            d += diff * self.m_per_tick
        self.total_dist += (d / 4.0)
        self.prev_ticks = curr

    # ─────────────────────────────────────────────────────
    #  Dual Ultrasonic PID Logic
    # ─────────────────────────────────────────────────────
    def _calculate_dual_pid(self):
        """หัวใจสำคัญ: คำนวณ PID จาก Ultrasonic 2 ตัว"""
        if not self.us_valid:
            return 0.0, 0.0  # ถ้า Sensor หลุด ให้วิ่งตรง

        # 1. Distance Error (ระยะห่างเฉลี่ย เทียบกับเป้า 35cm)
        avg_dist = (self.us_front + self.us_rear) / 2.0
        err_dist = self.US_TARGET_DIST - avg_dist 
        # ผล: ถ้าเป็นบวก (+) แปลว่าชิดไป -> ต้องเลี้ยวขวาออก
        
        # 2. Angle Error (ความเอียง: หน้า - หลัง)
        # Front < Rear (ติดลบ) = หัวปักเข้า -> ต้องเลี้ยวขวา (PID ต้องบวก)
        # Front > Rear (บวก)   = หัวบานออก -> ต้องเลี้ยวซ้าย (PID ต้องลบ)
        diff_angle = self.us_front - self.us_rear
        
        # รวมร่าง PID: 
        # (Distance Term) + (Angle Term * -1 เพื่อกลับทิศ)
        pid_out = (err_dist * self.KP_US_DIST) + (diff_angle * self.KP_US_ANGLE * -1.0)
        
        # Limit Output (ไม่ให้สะบัดแรงเกินไป)
        max_turn = self.walk_speed * 0.8
        pid_out = max(-max_turn, min(max_turn, pid_out))
        
        return pid_out, avg_dist

    # ─────────────────────────────────────────────────────
    #  Motor Helpers
    # ─────────────────────────────────────────────────────
    def _pid_to_rpm(self, pid_out):
        """แปลง PID เป็นความเร็วล้อซ้ายขวา"""
        # pid > 0 : เลี้ยวขวา (ล้อซ้ายหมุนเร็วกว่า)
        v_l = self.walk_speed + pid_out
        v_r = self.walk_speed - pid_out
        
        rpm_l = (v_l / (2 * math.pi * self.wheel_radius)) * 60
        rpm_r = (v_r / (2 * math.pi * self.wheel_radius)) * 60
        return rpm_l, rpm_r

    def _fwd_rpm(self):
        r = (self.walk_speed / (2 * math.pi * self.wheel_radius)) * 60
        return r, r

    def _back_rpm(self):
        r = (self.walk_speed * 0.5 / (2 * math.pi * self.wheel_radius)) * 60
        return -r, -r

    def _set_scan_mode(self, mode: float):
        msg = Float32(); msg.data = mode; self.scan_mode_pub.publish(msg)
        
    def send_rpm(self, l, r):
        msg = Twist()
        msg.linear.x = float(l); msg.angular.x = float(l)
        msg.linear.y = float(r); msg.angular.y = float(r)
        self.cmd_pub.publish(msg)

    # ─────────────────────────────────────────────────────
    #  Main Control Loop
    # ─────────────────────────────────────────────────────
    def control_loop(self):
        left_rpm  = 0.0
        right_rpm = 0.0
        pid_out   = 0.0

        # ── 1. SEARCH TAG ──
        if self.state == STATE_SEARCH:
            if self.tag_found:
                self.get_logger().info("Tag Found -> APPROACH")
                self.state = STATE_APPROACH
            else:
                left_rpm, right_rpm = self._fwd_rpm()

        # ── 2. APPROACH TAG ──
        elif self.state == STATE_APPROACH:
            is_close = (self.current_z_dist > 0.0 and self.current_z_dist < self.stop_distance_threshold)
            
            if self.tag_found and is_close:
                self.get_logger().info(f"Arrived at Tag (Z:{self.current_z_dist:.2f}) -> PAUSE")
                self.pause_start_time = time.monotonic()
                self.state = STATE_PAUSE
                self.send_rpm(0, 0); return
            elif not self.tag_found:
                self.state = STATE_SEARCH
            else:
                # Simple Vision PID (เลี้ยงTagให้อยู่กลาง)
                pid_out = self.tag_err * self.kp_tag
                left_rpm, right_rpm = 100.0 - pid_out, 100.0 + pid_out

        # ── 3. PAUSE ──
        elif self.state == STATE_PAUSE:
            if time.monotonic() - self.pause_start_time > 2.0:
                self.turn_start_time = time.monotonic()
                self.state = STATE_TURNL

        # ── 4. TURN LEFT ──
        elif self.state == STATE_TURNL:
            if time.monotonic() - self.turn_start_time < 7.0: # ตั้งเวลาเลี้ยวตามจริง
                left_rpm, right_rpm = -40.0, 40.0
            else:
                self.get_logger().info("Turn Done -> FWD SCAN")
                self.forward_start_time = time.monotonic()
                self.state = STATE_FORWARD_SCAN

        # ── 5. FORWARD SCAN (หาผนัง) ──
        elif self.state == STATE_FORWARD_SCAN:
            if time.monotonic() - self.forward_start_time < 2.0:
                left_rpm, right_rpm = self._fwd_rpm()
            else:
                # ถ้า Ultrasonic เจอผนังทั้งคู่ -> เริ่มจัดระเบียบ (Align)
                if self.us_valid: 
                    self.get_logger().info("Wall Detected -> ALIGN")
                    self.total_dist = 0.0
                    self.state = STATE_ALIGN_WALL
                else:
                    self.get_logger().info("Scanning for wall...")
                    left_rpm, right_rpm = -10.0, 10.0 # หมุนหาเบาๆ

        # ── 6. ALIGN WALL (Modified for Steep Angle) ──
        elif self.state == STATE_ALIGN_WALL:
            # 1. เช็คความต่างของระยะ (หน้า vs หลัง)
            diff = self.us_front - self.us_rear
            
            # 2. ตั้งค่า Threshold ความต่างที่ยอมรับได้ (เช่น 10-15 cm)
            # ถ้าต่างกันมากกว่านี้ แปลว่าเอียงกะเท่เร่ -> ต้องหมุนตัวเปล่าๆ ก่อน
            PIVOT_THRESHOLD = 0.15 

            if abs(diff) > PIVOT_THRESHOLD:
                self.get_logger().info(f"STEEP ANGLE! Diff:{diff:.2f} -> PIVOT TURN")
                
                # ถ้า Front < Rear (ติดลบ) = หัวปักเข้า -> ต้องหมุนขวา
                # ถ้า Front > Rear (บวก)   = หัวบานออก -> ต้องหมุนซ้าย
                if self.us_front < self.us_rear:
                    # หมุนขวาอยู่กับที่ (ล้อซ้ายเดินหน้า, ล้อขวาถอยหลัง)
                    left_rpm, right_rpm = 30.0, -30.0
                else:
                    # หมุนซ้ายอยู่กับที่
                    left_rpm, right_rpm = -30.0, 30.0
                
                # รีเซ็ตระยะทาง เพราะเราหมุนอยู่กับที่ ไม่นับว่าเดิน
                self.total_dist = 0.0

            else:
                # 3. ถ้าความต่างน้อยแล้ว (เริ่มขนาน) -> ใช้ Dual PID เดินหน้าปรับละเอียดต่อ
                pid_out, avg_dist = self._calculate_dual_pid()
                left_rpm, right_rpm = self._pid_to_rpm(pid_out)

                self.get_logger().info(f"FINE TUNE | Dist:{avg_dist:.2f} PID:{pid_out:.2f}", throttle_duration_sec=0.5)

                # เช็คเงื่อนไขจบ (ต้องขนานจริงๆ และระยะได้)
                is_parallel = abs(diff) < 0.03       # ต่างกันไม่เกิน 3cm
                is_dist_ok  = abs(avg_dist - self.US_TARGET_DIST) < 0.05
                
                if abs(self.total_dist) >= 0.20 and is_parallel and is_dist_ok:
                    self.get_logger().info("Aligned OK -> PREPARE BACK SCAN")
                    self.pause_align_start_time = time.monotonic()
                    self.state = STATE_PAUSE_AFTER_ALIGN
                    
        # ── 7. PAUSE BEFORE BACK ──
        elif self.state == STATE_PAUSE_AFTER_ALIGN:
            left_rpm, right_rpm = 0.0, 0.0
            if time.monotonic() - self.pause_align_start_time > 2.0:
                self._set_scan_mode(1.0) # เปิดโหมด Vision หลัง
                self.state = STATE_BACK_TO_TAG

        # ── 8. BACK TO START TAG ──
        elif self.state == STATE_BACK_TO_TAG:
            # ถอยจนกว่าจะเจอ Tag อยู่ขอบบนของจอ
            tag_at_top = (self.back_tag_found and self.back_tag_y_ratio < self.back_tag_threshold)
            
            if tag_at_top:
                self.get_logger().info("Back Tag Found at Top -> START RUNNING")
                self.send_rpm(0, 0)
                self._set_scan_mode(0.0) # ปิดโหมด Vision หลัง
                self.total_dist = 0.0
                self.state = STATE_GOTO_FIRST
            else:
                left_rpm, right_rpm = self._back_rpm()

        # ── 9. GOTO FIRST (Wall Following) ──
        elif self.state == STATE_GOTO_FIRST:
            target_distance = max(self.target_m, 0.5)
            self.get_logger().info(f"RUNNING | Dist:{self.total_dist:.2f}/{target_distance:.2f}", throttle_duration_sec=0.5)
            
            if abs(self.total_dist) >= target_distance:
                self.state = STATE_WAIT
            else:
                # ใช้ Dual PID เดินเลาะผนังยาวๆ
                pid_out, _ = self._calculate_dual_pid()
                left_rpm, right_rpm = self._pid_to_rpm(pid_out)

        # ── 10. WAIT ──
        elif self.state == STATE_WAIT:
            left_rpm, right_rpm = 0.0, 0.0

        # Output to Motors
        self.send_rpm(left_rpm, right_rpm)

        # Debug PID
        msg_p = Float32(); msg_p.data = float(pid_out)
        self.debug_pid_pub.publish(msg_p)

def main():
    rclpy.init()
    node = RobotMaster()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()