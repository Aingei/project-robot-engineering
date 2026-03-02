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
STATE_FORWARD_AFTER_TURN  = 5
STATE_SEARCH_WALL_ROTATE  = 8
STATE_ALIGN_WALL          = 4
STATE_PAUSE_AFTER_ALIGN   = 7
STATE_BACK_TO_TAG         = 6
STATE_GOTO_FIRST          = 2
STATE_WAIT                = 99

class RobotMaster(Node):
    def __init__(self):
        super().__init__('robot_master')

        # ── Robot Geometry ──────────────────────────────────────
        self.wheel_radius   = 0.06
        self.walk_speed     = 0.4
        self.align_speed    = 0.15  # ความเร็วตอนจัดระเบียบ
        self.ticks_per_rev  = 7436
        self.m_per_tick     = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev

        self.stop_distance_threshold = 0.5

        # ── Ultrasonic Config (Target 10cm) ───────────────────
        self.US_TARGET_DIST = 0.02
        
        # [TUNING] ค่าชดเชยเซนเซอร์ (สำคัญมาก!)
        # ถ้าวางขนานแล้วหน้าอ่านน้อยกว่าหลัง ให้ใส่ค่าบวก (เช่น 0.05)
        # ถ้าวางขนานแล้วหน้าอ่านมากกว่าหลัง ให้ใส่ค่าลบ (เช่น -0.02)
        self.FRONT_OFFSET = -0.015  # <--- ลองปรับเลขนี้ดูครับ
        
        self.KP_US_DIST  = 3.0      
        self.KP_US_ANGLE = 3.5      # เพิ่มแรงบิดให้หัวตรงมากขึ้น
        self.PIVOT_THRESHOLD = 0.15

        self.kp_tag  = 0.45
        self.state   = STATE_SEARCH
        self.target_m = 0.5

        # Vars
        self.us_front = 0.0; self.us_rear = 0.0; self.us_valid = False
        self.us_front_buf = []; self.us_rear_buf = []
        self.US_FILTER_N = 5
        self.lost_wall_count = 0
        
        self.tag_found = False; self.tag_err = 0.0; self.current_z_dist = 0.0
        self.back_tag_found = False; self.back_tag_y_ratio = 1.0; self.back_tag_threshold = 0.25
        self.total_dist = 0.0; self.prev_ticks = [0,0,0,0]; self.first_run = True
        self.pause_start_time = 0.0; self.turn_start_time = 0.0
        self.forward_start_time = 0.0; self.pause_align_start_time = 0.0
        self.back_start_time = 0.0

        self.cmd_pub = self.create_publisher(Twist, "/galum/cmd_move/rpm", 10)
        self.scan_mode_pub = self.create_publisher(Float32, "/galum/scan_mode", 10)
        self.create_subscription(Float32MultiArray, "/galum/us_dual", self.us_cb, qos.qos_profile_sensor_data)
        self.create_subscription(Float32MultiArray, "/galum/vision_packet", self.vision_cb, qos.qos_profile_sensor_data)
        self.create_subscription(Twist, "/galum/encoder", self.encoder_cb, qos.qos_profile_sensor_data)
        self.debug_pid_pub = self.create_publisher(Float32, "/debug/pid_output", 10)
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"Robot Master: Offset Mode (Front + {self.FRONT_OFFSET})")

    # ─────────────────────────────────────────────────────
    #  Callbacks
    # ─────────────────────────────────────────────────────
    def us_cb(self, msg):
        if len(msg.data) >= 2:
            f, r = msg.data[0], msg.data[1]
            if 0.02 < f < 2.5 and 0.02 < r < 2.5:
                self.us_front_buf.append(f)
                self.us_rear_buf.append(r)
                if len(self.us_front_buf) > self.US_FILTER_N:
                    self.us_front_buf.pop(0); self.us_rear_buf.pop(0)
                
                self.us_front = sum(self.us_front_buf) / len(self.us_front_buf)
                self.us_rear  = sum(self.us_rear_buf)  / len(self.us_rear_buf)
                self.us_valid = True
                self.lost_wall_count = 0
            else:
                self.lost_wall_count += 1
                if self.lost_wall_count > 5: self.us_valid = False

    def vision_cb(self, msg):
        d = msg.data
        self.tag_found = (d[0] == 1.0); self.tag_err = d[1]
        if self.tag_found and len(d) > 4 and d[4] > 0 and self.state == STATE_SEARCH:
            self.target_m = d[4] / 100.0
        self.current_z_dist = d[7] if len(d) > 7 else 0.0
        self.back_tag_found = (d[12] == 1.0) if len(d) > 12 else False
        self.back_tag_y_ratio = d[13] if len(d) > 13 else 1.0

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
    #  PID Logic (Update with Offset)
    # ─────────────────────────────────────────────────────
    def _calculate_dual_pid(self):
        if not self.us_valid: return 0.0, 0.0
        
        real_front = self.us_front + self.FRONT_OFFSET
        real_rear  = self.us_rear
        
        avg_dist = (real_front + real_rear) / 2.0
        
        err_dist = avg_dist - self.US_TARGET_DIST 
        diff_angle = real_front - real_rear
        
        pid_out = (err_dist * self.KP_US_DIST) + (diff_angle * self.KP_US_ANGLE)
        
        max_turn = self.align_speed * 0.8
        pid_out = max(-max_turn, min(max_turn, pid_out))
        return pid_out, avg_dist
    

    def _pid_to_rpm(self, pid_out, base_speed):
        v_l = base_speed + pid_out
        v_r = base_speed - pid_out
        return self._m_s_to_rpm(v_l), self._m_s_to_rpm(v_r)

    def _m_s_to_rpm(self, v):
        return (v / (2 * math.pi * self.wheel_radius)) * 60

    def _fwd_rpm(self): r = self._m_s_to_rpm(self.walk_speed); return r, r
    def _back_rpm(self): r = self._m_s_to_rpm(self.walk_speed * 0.5); return -r, -r
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
            if time.monotonic() - self.turn_start_time < 5.7: left_rpm, right_rpm = -40.0, 40.0
            else:
                self.get_logger().info("Turn done → FWD_WALK")
                self.forward_start_time = time.monotonic(); self.state = STATE_FORWARD_AFTER_TURN

        elif self.state == STATE_FORWARD_AFTER_TURN:
            if time.monotonic() - self.forward_start_time < 3.0:
                rpm = self._m_s_to_rpm(self.walk_speed); left_rpm, right_rpm = rpm, rpm
            else:
                self.get_logger().info("Walk Done -> Rotating RIGHT to find wall")
                self.state = STATE_SEARCH_WALL_ROTATE

        elif self.state == STATE_SEARCH_WALL_ROTATE:
            if self.us_valid:
                self.get_logger().info(f"Found! -> ALIGN")
                self.total_dist = 0.0; self.state = STATE_ALIGN_WALL
            else:
                left_rpm, right_rpm = 30.0, -30.0 # หมุนขวา

        elif self.state == STATE_ALIGN_WALL:
            if not self.us_valid:
                self.get_logger().warn("Lost Wall! -> Back to Rotate Search")
                self.state = STATE_SEARCH_WALL_ROTATE
            else:
                pid_out, avg_dist = self._calculate_dual_pid()
                left_rpm, right_rpm = self._pid_to_rpm(pid_out, self.align_speed)
                
                real_front = self.us_front + self.FRONT_OFFSET # ใช้ค่าที่แก้ Offset แล้วมาเช็ค
                diff = real_front - self.us_rear
                
                is_parallel = abs(diff) < 0.03
                is_dist_ok  = abs(avg_dist - self.US_TARGET_DIST) < 0.05
                
                if abs(self.total_dist) >= 0.20 and is_parallel and is_dist_ok:
                    self.get_logger().info("Aligned OK -> Back Scan")
                    self.pause_align_start_time = time.monotonic()
                    self.state = STATE_PAUSE_AFTER_ALIGN
                
                self.get_logger().info(f"ALIGN | Dist:{avg_dist:.2f} Diff:{diff:.2f} PID:{pid_out:.2f}", throttle_duration_sec=0.5)
        

        elif self.state == STATE_PAUSE_AFTER_ALIGN:
            left_rpm, right_rpm = 0.0, 0.0
            if time.monotonic() - self.pause_align_start_time > 2.0:
                self._set_scan_mode(1.0)
                self.back_start_time = time.monotonic()
                self.back_tag_found = False
                self.state = STATE_BACK_TO_TAG

        # ── 8. BACK TO TAG (Blind Back First) ──
        elif self.state == STATE_BACK_TO_TAG:
            elapsed_back = time.monotonic() - self.back_start_time
            
            # [LOGIC] 2 วินาทีแรก ถอยอย่างเดียว ไม่สนกล้อง (Blind Back)
            if elapsed_back < 2.0:
                rpm = self._m_s_to_rpm(self.walk_speed * 0.5)
                left_rpm, right_rpm = -rpm, -rpm
                self.get_logger().info(f"Blind Backing... ({elapsed_back:.1f}s)", throttle_duration_sec=0.5)
            
            else:
                # หลังจาก 2 วิ ค่อยเริ่มเช็ค Tag
                tag_ready = (self.back_tag_found and self.back_tag_y_ratio < self.back_tag_threshold)
                if tag_ready:
                    self.get_logger().info("Back Tag Found at Top -> START RUNNING WITH PID")
                    self.send_rpm(0, 0); self._set_scan_mode(0.0)
                    self.total_dist = 0.0; self.state = STATE_GOTO_FIRST
                else:
                    rpm = self._m_s_to_rpm(self.walk_speed * 0.5)
                    left_rpm, right_rpm = -rpm, -rpm
                    self.get_logger().info(f"Searching Back Tag...", throttle_duration_sec=0.5)

        # ── 9. GOTO FIRST (ใช้ PID เลี้ยงขนาน) ──
        elif self.state == STATE_GOTO_FIRST:
            target_limit = max(self.target_m, 0.5)
            
            if abs(self.total_dist) >= target_limit:
                self.state = STATE_WAIT
            else:
                # [NEW] ใช้ PID เลี้ยงขนานขณะวิ่ง
                pid_out, _ = self._calculate_dual_pid()
                # วิ่งความเร็วปกติ + PID
                left_rpm, right_rpm = self._pid_to_rpm(pid_out, self.walk_speed)
                self.get_logger().info(f"RUNNING PID | Dist:{self.total_dist:.2f}/{target_limit:.2f}", throttle_duration_sec=0.5)

        elif self.state == STATE_WAIT:
            left_rpm, right_rpm = 0.0, 0.0

        self.send_rpm(left_rpm, right_rpm)
        msg_p = Float32(); msg_p.data = float(pid_out); self.debug_pid_pub.publish(msg_p)

def main():
    rclpy.init()
    node = RobotMaster()
    try: rclpy.spin(node)
    except KeyboardInterrupt: pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()