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
STATE_ALIGN_ROW           = 4
STATE_PAUSE_AFTER_ALIGN   = 7
STATE_BACK_TO_TAG         = 6   # [NEW] ถอยหลังจนเจอ tag ขอบบน
STATE_GOTO_FIRST          = 2
STATE_WAIT                = 99

STATE_NAMES = {
    STATE_SEARCH: "SEARCH", STATE_APPROACH: "APPROACH",
    STATE_PAUSE: "PAUSE", STATE_TURNL: "TURNL",
    STATE_FORWARD_AFTER_TURN: "FWD_AFTER_TURN",
    STATE_ALIGN_ROW: "ALIGN_ROW",
    STATE_PAUSE_AFTER_ALIGN: "PAUSE_AFTER_ALIGN",
    STATE_BACK_TO_TAG: "BACK_TO_TAG",
    STATE_GOTO_FIRST: "GOTO_FIRST",
    STATE_WAIT: "WAIT"
}


class RobotMaster(Node):
    def __init__(self):
        super().__init__('robot_master')

        # ── Robot geometry ──────────────────────────────────────
        self.wheel_radius   = 0.05
        self.ticks_per_rev  = 7436
        self.m_per_tick     = (2 * math.pi * self.wheel_radius) / self.ticks_per_rev
        self.walk_speed     = 0.4

        # ── Thresholds ──────────────────────────────────────────
        self.stop_distance_threshold = 0.5
        self.align_lat_tol           = 80.0 #30
        self.align_ang_tol           = 999.0 #8
        self.align_min_dist          = 0.20

        # ── PID Gains ───────────────────────────────────────────
        self.kp_tag  = 0.45
        self.kp_lat  = 0.003
        self.kp_head = 0.0

        # ── Target offset ───────────────────────────────────────
        self.target_offset = -20

        # [NEW] Back scan threshold
        # tag_y_ratio < ค่านี้ = tag อยู่ขอบบนแล้ว → หยุดถอย
        self.back_tag_y_threshold = 0.25  # บน 25% ของภาพ

        # ── State variables ─────────────────────────────────────
        self.state    = STATE_SEARCH
        self.target_m = 0.5

        # Vision data
        self.tag_found      = False
        self.tag_err        = 0.0
        self.tag_size       = 0.0
        self.current_z_dist = 0.0
        self.yolo_found     = False
        self.lane_x         = 0.0
        self.lane_angle     = 0.0

        # [NEW] Back scan data
        self.back_tag_found   = False
        self.back_tag_y_ratio = 1.0

        # Encoder
        self.total_dist = 0.0
        self.prev_ticks = [0, 0, 0, 0]
        self.first_run  = True

        # Timers
        self.pause_start_time       = 0.0
        self.turn_start_time        = 0.0
        self.forward_start_time     = 0.0
        self.pause_align_start_time = 0.0

        # ── ROS I/O ─────────────────────────────────────────────
        self.cmd_pub = self.create_publisher(Twist, "/galum/cmd_move/rpm", 10)

        # [NEW] publisher สำหรับ scan_mode → บอก vision node
        self.scan_mode_pub = self.create_publisher(Float32, "/galum/scan_mode", 10)

        self.create_subscription(Float32MultiArray, "/galum/vision_packet",
                                 self.vision_cb, qos.qos_profile_sensor_data)
        self.create_subscription(Twist, "/galum/encoder",
                                 self.encoder_cb, qos.qos_profile_sensor_data)

        self.debug_err_pub = self.create_publisher(Float32, "/debug/error_input", 10)
        self.debug_pid_pub = self.create_publisher(Float32, "/debug/pid_output",  10)

        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Robot Master Started")

    # ─────────────────────────────────────────────────────
    #  Callbacks
    # ─────────────────────────────────────────────────────
    def vision_cb(self, msg):
        d = msg.data
        self.tag_found  = (d[0] == 1.0)
        self.tag_err    = d[1]
        self.tag_size   = d[2]
        self.lane_x     = d[3]

        if self.tag_found and len(d) > 4 and d[4] > 0:
            if self.state == STATE_SEARCH:
                raw_val = d[4]
                self.target_m = raw_val / 100.0
                self.get_logger().info(
                    f"target_m set: {self.target_m:.3f}m (raw d[4]={raw_val})",
                    throttle_duration_sec=1.0)

        self.current_z_dist = d[7]  if len(d) > 7  else 0.0
        self.yolo_found     = (d[8] == 1.0) if len(d) > 8  else True
        self.lane_angle     = d[9]  if len(d) > 9  else 0.0

        # [NEW] รับ back scan data index 12, 13
        self.back_tag_found   = (d[12] == 1.0) if len(d) > 12 else False
        self.back_tag_y_ratio = d[13]           if len(d) > 13 else 1.0

    def encoder_cb(self, msg):
        curr = [msg.linear.x, msg.linear.y, msg.angular.x, msg.angular.y]
        if self.first_run:
            self.prev_ticks = curr
            self.first_run  = False
            return
        d = 0.0
        for i in range(4):
            diff = curr[i] - self.prev_ticks[i]
            if i == 0 or i == 3:
                diff = -diff
            d += diff * self.m_per_tick
        self.total_dist += (d / 4.0)
        self.prev_ticks = curr

    # ─────────────────────────────────────────────────────
    #  Helpers
    # ─────────────────────────────────────────────────────
    def _lane_pid(self):
        lat_error  = self.lane_x - self.target_offset
        head_error = self.lane_angle

        pid_out = (lat_error * self.kp_lat) + (head_error * self.kp_head)
        max_turn = self.walk_speed * 0.8
        pid_out  = max(-max_turn, min(max_turn, pid_out))

        self.get_logger().info(
            f"PID | lane_x:{self.lane_x:+.0f} offset:{self.target_offset} "
            f"lat_err:{lat_error:+.0f} pid:{pid_out:+.3f}",
            throttle_duration_sec=0.3)

        return pid_out, lat_error

    def _lane_to_rpm(self, pid_out):
        v_l = self.walk_speed + pid_out
        v_r = self.walk_speed - pid_out
        rpm_l = (v_l / (2 * math.pi * self.wheel_radius)) * 60
        rpm_r = (v_r / (2 * math.pi * self.wheel_radius)) * 60
        return rpm_l, rpm_r

    def _fwd_rpm(self):
        r = (self.walk_speed / (2 * math.pi * self.wheel_radius)) * 60
        return r, r

    def _back_rpm(self):
        """ถอยหลังช้าๆ"""
        r = (self.walk_speed * 0.5 / (2 * math.pi * self.wheel_radius)) * 60
        return -r, -r  # ลบ = ถอยหลัง

    def _set_scan_mode(self, mode: float):
        msg = Float32()
        msg.data = mode
        self.scan_mode_pub.publish(msg)

    # ─────────────────────────────────────────────────────
    #  Main control loop (20 Hz)
    # ─────────────────────────────────────────────────────
    def control_loop(self):
        left_rpm  = 0.0
        right_rpm = 0.0
        error_in  = 0.0
        pid_out   = 0.0

        # ── 1. SEARCH ─────────────────────────────────────
        if self.state == STATE_SEARCH:
            if self.tag_found:
                self.get_logger().info("Tag found → APPROACH")
                self.state = STATE_APPROACH
            else:
                left_rpm, right_rpm = self._fwd_rpm()

        # ── 2. APPROACH ───────────────────────────────────
        elif self.state == STATE_APPROACH:
            stop_ok = (self.current_z_dist > 0.0 and
                       self.current_z_dist < self.stop_distance_threshold)
            if self.tag_found and stop_ok:
                self.get_logger().info(f"Arrived Z:{self.current_z_dist:.2f}m → PAUSE")
                self.pause_start_time = time.monotonic()
                self.state = STATE_PAUSE
                self.send_rpm(0, 0)
                return
            elif not self.tag_found:
                self.state = STATE_SEARCH
            else:
                error_in  = self.tag_err
                pid_out   = self.tag_err * self.kp_tag
                left_rpm  = 100.0 - pid_out
                right_rpm = 100.0 + pid_out

        # ── 3. PAUSE ──────────────────────────────────────
        elif self.state == STATE_PAUSE:
            left_rpm = right_rpm = 0.0
            if time.monotonic() - self.pause_start_time > 2.0:
                self.get_logger().info("Pause done → TURNL")
                self.turn_start_time = time.monotonic()
                self.state = STATE_TURNL

        # ── 4. TURN LEFT ──────────────────────────────────
        elif self.state == STATE_TURNL:
            elapsed = time.monotonic() - self.turn_start_time
            if elapsed < 7.0:
                left_rpm, right_rpm = -40.0, 40.0
            else:
                self.get_logger().info("Turn done → FWD_AFTER_TURN")
                self.forward_start_time = time.monotonic()
                self.state = STATE_FORWARD_AFTER_TURN

        # ── 5. FORWARD (เดินเข้าหาแปลง) ────────────────────
        elif self.state == STATE_FORWARD_AFTER_TURN:
            elapsed = time.monotonic() - self.forward_start_time
            if elapsed < 2.0:
                left_rpm, right_rpm = self._fwd_rpm()
            else:
                if self.yolo_found:
                    self.get_logger().info("Fwd done, row found → ALIGN_ROW")
                    self.total_dist = 0.0
                    self.state = STATE_ALIGN_ROW
                else:
                    self.get_logger().info("Fwd done, no row → scanning")
                    left_rpm, right_rpm = -10.0, 10.0

        # ── 6. ALIGN ROW ──────────────────────────────────
        elif self.state == STATE_ALIGN_ROW:
            if not self.yolo_found:
                left_rpm = right_rpm = 0.0
            else:
                pid_out, lat_error = self._lane_pid()
                left_rpm, right_rpm = self._lane_to_rpm(pid_out)
                error_in = lat_error

                self.get_logger().info(
                    f"ALIGN | lane_x:{self.lane_x:+.0f} lat_err:{lat_error:+.0f} "
                    f"ang:{self.lane_angle:.1f} pid:{pid_out:.3f}",
                    throttle_duration_sec=0.5)

                if (abs(self.total_dist) >= self.align_min_dist and
                        abs(lat_error) < self.align_lat_tol and
                        abs(self.lane_angle) < self.align_ang_tol):
                    self.get_logger().info("Aligned! → PAUSE_AFTER_ALIGN")
                    self.pause_align_start_time = time.monotonic()
                    self.state = STATE_PAUSE_AFTER_ALIGN

        # ── 7. PAUSE AFTER ALIGN ──────────────────────────
        elif self.state == STATE_PAUSE_AFTER_ALIGN:
            left_rpm, right_rpm = 0.0, 0.0
            if time.monotonic() - self.pause_align_start_time > 2.0:
                # [NEW] เปิด scan mode แล้วไป BACK_TO_TAG
                self.get_logger().info("Aligned! → BACK_TO_TAG (scan mode ON)")
                self._set_scan_mode(1.0)
                self.state = STATE_BACK_TO_TAG

        # ── [NEW] 8. BACK TO TAG ───────────────────────────
        elif self.state == STATE_BACK_TO_TAG:
            # ถ้าเจอ tag แล้วและอยู่ขอบบน → หยุดถอย ไป GOTO_FIRST
            tag_at_top = (self.back_tag_found and
                          self.back_tag_y_ratio < self.back_tag_y_threshold)

            if tag_at_top:
                self.get_logger().info(
                    f"Tag at top (y_ratio:{self.back_tag_y_ratio:.2f}) → GOTO_FIRST")
                self.send_rpm(0, 0)
                self._set_scan_mode(0.0)  # ปิด scan mode กลับเป็น YOLO
                self.total_dist = 0.0
                self.state = STATE_GOTO_FIRST
            else:
                # ถอยหลังช้าๆ
                left_rpm, right_rpm = self._back_rpm()
                self.get_logger().info(
                    f"BACK | tag_found:{self.back_tag_found} "
                    f"y_ratio:{self.back_tag_y_ratio:.2f} "
                    f"threshold:{self.back_tag_y_threshold}",
                    throttle_duration_sec=0.5)

        # ── 9. GOTO FIRST ─────────────────────────────────
        elif self.state == STATE_GOTO_FIRST:
            self.get_logger().info(
                f"GOTO | dist:{self.total_dist:.3f}m / {self._target_dist():.3f}m",
                throttle_duration_sec=0.5)
            if abs(self.total_dist) >= self._target_dist():
                self.get_logger().info("Reached target → WAIT")
                self.state = STATE_WAIT
            else:
                if self.yolo_found:
                    pid_out, lat_error = self._lane_pid()
                    error_in = lat_error
                else:
                    pid_out = 0.0
                left_rpm, right_rpm = self._lane_to_rpm(pid_out)

        # ── 10. WAIT ──────────────────────────────────────
        elif self.state == STATE_WAIT:
            left_rpm = right_rpm = 0.0

        # ── Publish ───────────────────────────────────────
        self.send_rpm(left_rpm, right_rpm)

        msg_e = Float32(); msg_e.data = float(error_in)
        msg_p = Float32(); msg_p.data = float(pid_out)
        self.debug_err_pub.publish(msg_e)
        self.debug_pid_pub.publish(msg_p)

    # ─────────────────────────────────────────────────────
    def _target_dist(self):
        MIN_DIST = 0.5
        return max(self.target_m, MIN_DIST)

    def send_rpm(self, l, r):
        msg = Twist()
        msg.linear.x  = float(l); msg.angular.x = float(l)
        msg.linear.y  = float(r); msg.angular.y = float(r)
        self.cmd_pub.publish(msg)


def main():
    rclpy.init()
    node = RobotMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()