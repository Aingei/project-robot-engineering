#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy import qos

class stepper(Node):
    def __init__(self):
        super().__init__("stepper")

        # โชว์ค่าบน /galum/stepper/angle (ตามโครงสร้างเดิม)
        self.send_robot_stepper = self.create_publisher(
            Twist, "/galum/stepper/angle", qos_profile=qos.qos_profile_system_default
        )

        # รับคำสั่งจาก /galum/stepper (เหมือนเดิม)
        self.create_subscription(
            Twist, "/galum/stepper", self.rotate_stepper,
            qos_profile=qos.qos_profile_system_default
        )

        # สถานะภายใน (เหมือนเดิม + เติมตัวแปรที่ใช้จริง)
        self.stepper_angle: float = 0.0
        self.current_speed: float = 0.0
        self._default_speed: float = 1600.0  # ปรับได้
        self._stop_at: float = None

        # timer ส่งทุก 10ms (อันเดียวพอ)
        self.create_timer(0.01, self.sendData)

    # ---------- รับคำสั่ง ----------
    def rotate_stepper(self, msg: Twist):
        x = float(msg.linear.x)

        if x == 1.0:
            self.cmd_stepper_speed = +float(self._default_speed , 5.0)  
        elif x == 2.0:
            self.cmd_stepper_speed = -float(self._default_speed , 5.0)
        elif x == 0.0:
            self.current_speed = 0.0
            self._stop_at = None
        else:
            # กรณีส่งความเร็วมาโดยตรง (เช่น 600 หรือ -1200)
            self.current_speed = x
            self._stop_at = None

        # โครงสร้างเดิมของคุณใช้ stepper_angle ในการ publish
        # ก็อัปเดตให้เท่ากับคำสั่งล่าสุดไปเลย
        self.stepper_angle = self.cmd_stepper_speed

        # debug สั้น ๆ (ปิดได้)
        # self.get_logger().info(f"[IN ] cmd={self.cmd_stepper_speed:.1f}")
        
    def cmd_stepper_speed(self, speed: float, seconds: float):
        self._current_speed = float(speed)
        # ตั้งเส้นตายหยุดที่เวลาปัจจุบัน + seconds
        now = self.get_clock().now()
        self._stop_at = now + Duration(seconds=float(seconds))

    # ---------- ส่งต่อไป /galum/stepper/angle ----------
    def sendData(self):
        
        if self._stop_at is not None and self.get_clock().now() >= self._stop_at:
            self._current_speed = 0.0
            self._stop_at = None
        
        stepper_msg = Twist()
        stepper_msg.linear.x = float(self.stepper_angle)
        self.send_robot_stepper.publish(stepper_msg)

        # debug สั้น ๆ (ปิดได้)
        # self.get_logger().info(f"[OUT] angle={stepper_msg.linear.x:.1f}")

def main(args=None):
    rclpy.init()
    sub = stepper()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
