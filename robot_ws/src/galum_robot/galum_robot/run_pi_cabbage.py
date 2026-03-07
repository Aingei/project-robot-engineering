#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float32 , String
from rclpy import qos
import math
import time

import matplotlib.pyplot as plt

# ─────────────────────────────────────────────────────
#  States
# ─────────────────────────────────────────────────────
STATE_SEARCH              = 0
STATE_APPROACH            = 1
STATE_PAUSE               = 15
STATE_TURNL               = 3
STATE_FORWARD_AFTER_TURN  = 5
STATE_SEARCH_WALL_ROTATE  = 8
STATE_ALIGN_WALL          = 4
STATE_PAUSE_AFTER_ALIGN   = 7
STATE_BACK_TO_TAG         = 6
STATE_BACK_BLIND          = 10  
STATE_RUN_SEQUENCE        = 11  
STATE_PAUSE_SEQUENCE      = 12  
STATE_PAUSE_BEFORE_SEQUENCE = 13 
STATE_WAIT                = 99

class RobotMaster(Node):
    def __init__(self):
        super().__init__('robot_master')

        # ── Robot Geometry ──────────────────────────────────────
        self.wheel_radius   = 0.06
        self.walk_speed     = 0.4
        self.align_speed    = 0.25
        self.ticks_per_rev  = 7436
        self.m_per_tick     = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev

        self.stop_distance_threshold = 0.48

        # ── Ultrasonic Config ─────────────────────────────────
        self.US_TARGET_DIST = 0.04 
        self.FRONT_OFFSET = -0.015
        self.REAR_OFFSET = 0.08
        
        # ── PID GAINS ─────────────────────────────────────────
        self.KP_US_DIST  = 5.0       
        self.KP_US_ANGLE = 4.5      
        self.KI_US = 0.1            
        self.KD_US = 0.8            
        
        self.align_start_time = 0.0
        self.integral_error = 0.0
        self.prev_total_error = 0.0 
        # ──────────────────────────────────────────────────────

        self.PIVOT_THRESHOLD = 0.15
        self.kp_tag  = 0.45
        self.state   = STATE_SEARCH

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
        
        # ── [เพิ่ม] ตัวแปรสำหรับกะหล่ำ ──
        self.cabbage_diameter = 0.0 
        self.logged_cabbage = False 
        
        self.pause_start_time = 0.0
        self.turn_start_time = 0.0
        self.forward_start_time = 0.0
        self.pause_align_start_time = 0.0
        self.back_start_time = 0.0
        self.blind_back_start_time = 0.0
        self.sequence_pause_start_time = 0.0

        self.sequence_steps = []
        self.current_step_index = 0

        self.history_time = []
        self.history_dist = []
        self.history_target = []
        self.start_plot_time = 0.0
        self.has_plotted = False 

        self.cmd_pub = self.create_publisher(Twist, "/galum/cmd_move/rpm", 10)
        self.scan_mode_pub = self.create_publisher(Float32, "/galum/scan_mode", 10)
        self.create_subscription(Float32MultiArray, "/galum/us_dual", self.us_cb, qos.qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, "/galum/vision_packet", self.vision_cb, qos.qos_profile_sensor_data)
        self.create_subscription(Twist, "/galum/encoder", self.encoder_cb, qos.qos_profile_sensor_data)
        self.debug_pid_pub = self.create_publisher(Float32, "/debug/pid_output", 10)
        
        # 🟢 [เพิ่มบรรทัดนี้] เอาไว้ส่งคำสั่งเซฟรูป
        self.save_img_pub = self.create_publisher(String, "/galum/save_image_cmd", 10)
        
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"Robot Master: Sequence Mode + Cabbage Scan")

    # ─────────────────────────────────────────────────────
    #  Callbacks
    # ─────────────────────────────────────────────────────
    def us_cb(self, msg):
        if len(msg.data) >= 2:
            f, r = msg.data[0], msg.data[1]
            if 0.005 < f < 2.5 and 0.005 < r < 2.5:
                self.us_front_buf.append(f)
                self.us_rear_buf.append(r)
                if len(self.us_front_buf) > self.US_FILTER_N:
                    self.us_front_buf.pop(0); self.us_rear_buf.pop(0)
                
                self.us_front = sum(self.us_front_buf) / len(self.us_front_buf)
                self.us_rear  = sum(self.us_rear_buf)  / len(self.us_rear_buf)
                self.us_valid = True
                self.lost_wall_count = 0
                
                self.current_avg_dist = (self.us_front + (self.us_rear + self.REAR_OFFSET)) / 2.0
            else:
                self.lost_wall_count += 1
                if self.lost_wall_count > 5: self.us_valid = False

    def vision_cb(self, msg):
        d = msg.data
        self.tag_found = (d[0] == 1.0)
        self.tag_err = d[1]
        
        if len(d) > 6:
            self.plant_dist_m     = d[4] / 100.0  
            self.plant_gap_m      = d[5] / 100.0  
            self.plant_interval_m = d[6] / 100.0  

        self.current_z_dist = d[7] if len(d) > 7 else 0.0
        self.back_tag_found = (d[12] == 1.0) if len(d) > 12 else False
        self.back_tag_y_ratio = d[13] if len(d) > 13 else 1.0
        
        # ── [เพิ่ม] รับค่าขนาดกะหล่ำจากกล้อง ──
        self.cabbage_diameter = d[14] if len(d) > 14 else 0.0

    def encoder_cb(self, msg):
        curr = [msg.linear.x, msg.linear.y, msg.angular.x, msg.angular.y]
        if self.first_run: self.prev_ticks = curr; self.first_run = False; return
        d = 0.0
        for i in range(4):
            diff = curr[i] - self.prev_ticks[i]
            if i == 0 or i == 3: diff = -diff
            d += diff * self.m_per_tick
        self.total_dist += (d / 4.0)
        self.prev_ticks = curr

    # ─────────────────────────────────────────────────────
    #  PID Logic (ไม่ถูกเปลี่ยนแปลง)
    # ─────────────────────────────────────────────────────
    def _calculate_dual_pid(self):
        if not self.us_valid: return 0.0, 0.0
        avg_dist = self.current_avg_dist
        err_dist = avg_dist - self.US_TARGET_DIST 
        
        real_front = self.us_front 
        real_rear  = self.us_rear + self.REAR_OFFSET
        diff_angle = real_front - real_rear
        
        total_error = (err_dist * self.KP_US_DIST) + (diff_angle * self.KP_US_ANGLE)
        self.integral_error += total_error
        self.integral_error = max(-0.1, min(0.1, self.integral_error)) 
        
        derivative = total_error - self.prev_total_error
        self.prev_total_error = total_error 
        
        pid_out = total_error + (self.integral_error * self.KI_US) + (derivative * self.KD_US)
        max_turn = self.align_speed * 0.8
        pid_out = max(-max_turn, min(max_turn, pid_out))
        return pid_out, avg_dist
    
    def _pid_to_rpm(self, pid_out, base_speed):
        v_l = base_speed + pid_out
        v_r = base_speed - pid_out
        return self._m_s_to_rpm(v_l), self._m_s_to_rpm(v_r)

    def _get_reverse_rpm(self, base_speed):
        if not self.us_valid:
            return self._m_s_to_rpm(base_speed), self._m_s_to_rpm(base_speed)
        
        real_front = self.us_front + self.FRONT_OFFSET
        real_rear  = self.us_rear
        diff_angle = real_front - real_rear
        err_dist = self.current_avg_dist - self.US_TARGET_DIST
        
        KP_REV_ANGLE = 6.0  
        KP_REV_DIST  = 1.5  
        
        pid_out = (diff_angle * KP_REV_ANGLE) - (err_dist * KP_REV_DIST)
        max_turn = self.align_speed * 0.6
        pid_out = max(-max_turn, min(max_turn, pid_out))
        
        v_l = base_speed + pid_out
        v_r = base_speed - pid_out
        return self._m_s_to_rpm(v_l), self._m_s_to_rpm(v_r)

    def _m_s_to_rpm(self, v):
        return (v / (2 * math.pi * self.wheel_radius)) * 60

    def _set_scan_mode(self, mode: float): msg = Float32(); msg.data = mode; self.scan_mode_pub.publish(msg)
    def send_rpm(self, l, r): msg = Twist(); msg.linear.x = float(l); msg.angular.x = float(l); msg.linear.y = float(r); msg.angular.y = float(r); self.cmd_pub.publish(msg)

    # ─────────────────────────────────────────────────────
    #  Main Loop
    # ─────────────────────────────────────────────────────
    def control_loop(self):
        left_rpm = 0.0; right_rpm = 0.0; pid_out = 0.0

        if self.state == STATE_SEARCH:
            if self.tag_found: self.state = STATE_APPROACH
            else: rpm = self._m_s_to_rpm(self.walk_speed); left_rpm, right_rpm = rpm, rpm

        elif self.state == STATE_APPROACH:
            is_close = (0.0 < self.current_z_dist < self.stop_distance_threshold)
            if self.tag_found and is_close:
                self.pause_start_time = time.monotonic(); self.state = STATE_PAUSE; self.send_rpm(0, 0); return
            elif not self.tag_found: self.state = STATE_SEARCH
            else: pid = self.tag_err * self.kp_tag; left_rpm, right_rpm = 60.0 - pid, 60.0 + pid

        elif self.state == STATE_PAUSE:
            if time.monotonic() - self.pause_start_time > 2.0:
                self.turn_start_time = time.monotonic(); self.state = STATE_TURNL

        elif self.state == STATE_TURNL:
            if time.monotonic() - self.turn_start_time < 6.3: left_rpm, right_rpm = -40.0, 40.0
            else:
                self.get_logger().info("Turn done → FWD_WALK")
                self.forward_start_time = time.monotonic(); self.state = STATE_FORWARD_AFTER_TURN

        elif self.state == STATE_FORWARD_AFTER_TURN:
            if time.monotonic() - self.forward_start_time < 2.7:
                rpm = self._m_s_to_rpm(self.walk_speed); left_rpm, right_rpm = rpm, rpm
            else:
                self.get_logger().info("Walk Done -> Rotating RIGHT to find wall")
                self.state = STATE_SEARCH_WALL_ROTATE

        elif self.state == STATE_SEARCH_WALL_ROTATE:
            if self.us_valid:
                self.get_logger().info(f"Found! -> ALIGN")
                self.total_dist = 0.0
                self.integral_error = 0.0
                self.prev_total_error = 0.0
                self.history_time.clear()
                self.history_dist.clear()
                self.history_target.clear()
                self.start_plot_time = time.monotonic()
                self.has_plotted = False
                self.align_start_time = time.monotonic()
                self.state = STATE_ALIGN_WALL
            else:
                left_rpm, right_rpm = 30.0, -30.0 

        elif self.state == STATE_ALIGN_WALL:
            if time.monotonic() - self.align_start_time > 20.0:
                self.pause_align_start_time = time.monotonic()
                self.state = STATE_PAUSE_AFTER_ALIGN
            elif not self.us_valid:
                self.state = STATE_SEARCH_WALL_ROTATE
            else:
                pid_out, avg_dist = self._calculate_dual_pid()
                left_rpm, right_rpm = self._pid_to_rpm(pid_out, self.align_speed)
                
                real_front = self.us_front + self.FRONT_OFFSET 
                diff = real_front - self.us_rear
                is_parallel = abs(diff) < 0.03
                is_dist_ok  = abs(avg_dist - self.US_TARGET_DIST) < 0.05
                
                if abs(self.total_dist) >= 0.20 and is_parallel and is_dist_ok:
                    self.pause_align_start_time = time.monotonic()
                    self.state = STATE_PAUSE_AFTER_ALIGN
                
        elif self.state == STATE_PAUSE_AFTER_ALIGN:
            left_rpm, right_rpm = 0.0, 0.0
            if time.monotonic() - self.pause_align_start_time > 2.0:
                self._set_scan_mode(1.0)
                self.back_start_time = time.monotonic()
                self.back_tag_found = False
                self.state = STATE_BACK_TO_TAG

        elif self.state == STATE_BACK_TO_TAG:
            elapsed_back = time.monotonic() - self.back_start_time
            back_speed = -self.walk_speed * 0.5
            
            if elapsed_back < 2.0:
                left_rpm, right_rpm = self._get_reverse_rpm(back_speed)
            else:
                tag_ready = (self.back_tag_found and self.back_tag_y_ratio < self.back_tag_threshold)
                if tag_ready:
                    self.sequence_steps = [
                        self.plant_dist_m,     
                        self.plant_dist_m,     
                        self.plant_gap_m,      
                        self.plant_interval_m, 
                        self.plant_interval_m, 
                        self.plant_interval_m  
                    ]
                    self.get_logger().info(f"Back Tag Found! Sequence Generated: {self.sequence_steps}")
                    
                    self.send_rpm(0, 0)
                    self._set_scan_mode(0.0)
                    
                    self.blind_back_start_time = time.monotonic()
                    self.state = STATE_BACK_BLIND
                else:
                    left_rpm, right_rpm = self._get_reverse_rpm(back_speed)

        elif self.state == STATE_BACK_BLIND:
            elapsed = time.monotonic() - self.blind_back_start_time
            if elapsed < 6.5:
                back_rpm = self._m_s_to_rpm(-self.walk_speed * 0.5)
                left_rpm, right_rpm = back_rpm, back_rpm
            else:
                self.get_logger().info("Blind Reverse Done -> PAUSE BEFORE SEQUENCE")
                self.send_rpm(0, 0)
                self.total_dist = 0.0
                self.current_step_index = 0
                self.pause_start_time = time.monotonic()
                self.state = STATE_PAUSE_BEFORE_SEQUENCE
        
        elif self.state == STATE_PAUSE_BEFORE_SEQUENCE:
            left_rpm, right_rpm = 0.0, 0.0
            if time.monotonic() - self.pause_start_time > 2.0:
                self.get_logger().info("START RUNNING SEQUENCE BLIND (STRAIGHT)")
                
                # ── [เพิ่ม] สั่งเปิดกล้องกะหล่ำล่วงหน้า (ถ้าก้าวแรกเป็น interval เลย) ──
                if self.current_step_index >= 3:
                    self._set_scan_mode(2.0)
                    
                self.state = STATE_RUN_SEQUENCE

        elif self.state == STATE_RUN_SEQUENCE:
            if self.current_step_index >= len(self.sequence_steps):
                self.get_logger().info("ALL SEQUENCES COMPLETED!")
                self._set_scan_mode(0.0) # ── [เพิ่ม] ปิดกล้องกะหล่ำ ──
                self.state = STATE_WAIT
            else:
                target_dist = self.sequence_steps[self.current_step_index]
                if abs(self.total_dist) >= target_dist:
                    self.get_logger().info(f"Finished step {self.current_step_index + 1} ({target_dist}m) -> PAUSE")
                    self.send_rpm(0, 0)
                    
                    # ── [เพิ่ม] เตรียมตัวเข้าสู่การหยุดพัก ──
                    self.sequence_pause_start_time = time.monotonic()
                    self.logged_cabbage = False 
                    self.state = STATE_PAUSE_SEQUENCE
                else:
                    base_rpm = self._m_s_to_rpm(self.walk_speed)
                    left_rpm, right_rpm = base_rpm, base_rpm

        elif self.state == STATE_PAUSE_SEQUENCE:
            left_rpm, right_rpm = 0.0, 0.0
            elapsed = time.monotonic() - self.sequence_pause_start_time
            
            # ── [เพิ่มคำสั่งเซฟรูปตรงนี้] ──
            if self.current_step_index >= 3 and elapsed > 1.0 and not self.logged_cabbage:
                plant_num = self.current_step_index - 2
                self.get_logger().info(f"🟢 ต้นที่ {plant_num} ขนาดกะหล่ำที่วัดได้: {self.cabbage_diameter:.2f} cm")
                self.logged_cabbage = True
                
                # สั่งกล้องให้เซฟรูป พร้อมตั้งชื่อไฟล์ให้เลย
                msg = String()
                msg.data = f"cabbage_plant_{plant_num}.jpg"
                self.save_img_pub.publish(msg)
            
            # ── [เพิ่ม] ปริ้นค่ากะหล่ำตอนหยุดนิ่ง ──
            if self.current_step_index >= 3 and elapsed > 1.0 and not self.logged_cabbage:
                plant_num = self.current_step_index - 2
                self.get_logger().info(f"🟢 ต้นที่ {plant_num} ขนาดกะหล่ำที่วัดได้: {self.cabbage_diameter:.2f} cm")
                self.logged_cabbage = True
                
            if elapsed > 2.0:
                self.total_dist = 0.0
                self.current_step_index += 1
                
                # ── [เพิ่ม] สลับกล้องไปสแกนกะหล่ำ ถ้าเข้าสู่จังหวะที่ >= 3 ──
                if self.current_step_index >= 3:
                    self._set_scan_mode(2.0)
                else:
                    self._set_scan_mode(0.0)
                    
                self.state = STATE_RUN_SEQUENCE

        elif self.state == STATE_WAIT:
            left_rpm, right_rpm = 0.0, 0.0
            if not self.has_plotted and len(self.history_time) > 0:
                self.has_plotted = True
                self.get_logger().info("Generating PID Plot...")
                self.show_and_save_plot()

        self.send_rpm(left_rpm, right_rpm)
        msg_p = Float32(); msg_p.data = float(pid_out); self.debug_pid_pub.publish(msg_p)

        if self.us_valid and (self.state == STATE_ALIGN_WALL or self.state == STATE_BACK_TO_TAG):
            current_t = time.monotonic() - self.start_plot_time
            self.history_time.append(current_t)
            self.history_dist.append(self.current_avg_dist)
            self.history_target.append(self.US_TARGET_DIST)

    def show_and_save_plot(self):
        plt.figure(figsize=(10, 5))
        plt.plot(self.history_time, self.history_dist, 'b-', label='Actual Avg Dist (m)', linewidth=2)
        plt.plot(self.history_time, self.history_target, 'r--', label='Target (0.02m)', linewidth=2)
        plt.title('Ultrasonic PID Performance')
        plt.xlabel('Time (seconds)')
        plt.ylabel('Distance (meters)')
        plt.grid(True)
        plt.legend()
        plt.savefig('pid_tuning_result.png')
        self.get_logger().info("Saved plot to 'pid_tuning_result.png'")
        # plt.show() 

def main():
    rclpy.init()
    node = RobotMaster()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()