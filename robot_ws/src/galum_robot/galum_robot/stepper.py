#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy import qos
from time import time

DEFAULT_PULSES_PER_CLICK = 50

class stepper(Node):
    def __init__(self):
        super().__init__("stepper")

        # Publish คำสั่งไป ESP32 (ใช้ topic เดิมตามที่ต้องการ)
        self.send_robot_stepper = self.create_publisher(
            Twist, "/galum/stepper/angle", qos_profile=qos.qos_profile_system_default
        )

        # Subscribe รับจากจอย
        self._sub = self.create_subscription(
            Twist, "/galum/stepper", self.on_cmd,
            qos_profile=qos.qos_profile_system_default
        )

        # Internal state
        self.last_dir_cw: bool = True
        self.last_pulses: int = 0
        self.last_cmd_ts: float = 0.0

        # คำสั่งที่รอส่ง (None = ไม่มีคำสั่งรอ)
        self._pending_msg: Twist | None = None

        # ให้ timer เรียกส่ง "เฉพาะเมื่อมีคำสั่งรอ"
        self.create_timer(0.01, self.sendData)

    # ---------- รับคำสั่ง ----------
    def on_cmd(self, msg: Twist):
        x = float(msg.linear.x)
        y = float(msg.linear.y)
        
        

        # 3.0 = CW, 4.0 = CCW; ค่าอื่นไม่ทำอะไร
        if msg.linear.x == 3.0:
            direction_cw = True
        elif msg.linear.x == 4.0:
            direction_cw = False
        else:
            return

        # จำนวน pulses (ใช้ y ถ้า >0 ไม่งั้น default)
        pulses = int(round(y)) if y > 0.0 else DEFAULT_PULSES_PER_CLICK
        if pulses <= 0:
            pulses = DEFAULT_PULSES_PER_CLICK

        # เตรียมแพ็กเก็ตสำหรับยิงไป ESP32 (one-shot)
        out = Twist()
        out.linear.x = 1.0 if direction_cw else -1.0   # +1 = CW, -1 = CCW
        out.linear.y = float(pulses)                   # pulses ต่อคลิก
        self._pending_msg = out

        # อัปเดตสถานะ (ไว้ดู/ดีบัก ถ้าต้องการ)
        self.last_dir_cw = direction_cw
        self.last_pulses = pulses
        self.last_cmd_ts = time()

        self.get_logger().info(f"[QUEUE] dir={'CW' if direction_cw else 'CCW'}, pulses={pulses}")

    # ---------- ส่งคำสั่ง (one-shot ผ่าน timer) ----------
    def sendData(self):
        if self._pending_msg is not None:
            self.send_robot_stepper.publish(self._pending_msg)
            self.get_logger().info("[→ESP32 /galum/stepper/angle] sent")
            self._pending_msg = None  # เคลียร์คิว ไม่ยิงซ้ำ

def main(args=None):
    rclpy.init()
    sub = stepper()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()