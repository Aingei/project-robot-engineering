#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy import qos
from time import time, monotonic  # ใช้ monotonic สำหรับ debounce timing

# DEFAULT_PULSES_PER_CLICK = 90   # จำนวนพัลส์ต่อคลิก ถ้า y <= 0 จะใช้ค่านี้
DEBOUNCE_S = 0.12               # กันเด้ง 120 ms (ปรับได้ 0.08–0.20)

STEPS_PER_REV = 200           # สเต็ปต่อรอบของมอเตอร์ 1.8° (200 ฟูลสเต็ป = 360°)
MICROSTEP     = 16            # ตั้งตามจัมเปอร์ DRV8825 (8, 16, 32, ...)
DEG_PER_CLICK = 50            # อยากให้ 50° ต่อคลิก (ปรับตามชอบ) (เดี๋ยวไปคำนวณเป็นพัลส์จริงด้วย deg_to_pulses)

def deg_to_pulses(deg: float) -> int:
    pulses = int(round((STEPS_PER_REV * MICROSTEP) * (deg / 360.0)))
    return max(1, pulses)
#200 × 16 = 3200 ไมโครสเต็ป/รอบ 50°/คลิก ⇒ 3200 × (50/360) = 3200 × 0.138888... ≈ 444.444 → ปัดเป็น 444 พัลส์/คลิก

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

        # Internal state (สำหรับ monitor)
        self.last_dir_cw: bool = True
        self.last_pulses: int = 0
        self.last_cmd_ts: float = 0.0

        # Queue ส่งคำสั่ง (None = ไม่มีคำสั่งรอ)
        self._pending_msg: Twist | None = None

        # สถานะสำหรับ edge-detect + debounce
        self._pressed: bool = False        # กำลังกดอยู่ไหม (ตามมุมมองเรา)
        self._last_sign: int = 0           # -1=CCW, +1=CW, 0=ปล่อย
        self._last_click_ts: float = 0.0   # เวลา trigger ล่าสุด (monotonic)

        # ให้ timer เรียกส่ง "เฉพาะเมื่อมีคำสั่งรอ"
        self.create_timer(0.01, self.sendData)

    # ---------- รับคำสั่งจากจอย ----------
    def on_cmd(self, msg: Twist):
        # map ปุ่ม: 3.0 = CW, 4.0 = CCW, ค่าอื่น = ปล่อยปุ่ม
        if msg.linear.x == 3.0: 
            sign = +1   # CW
        elif msg.linear.x == 4.0:
            sign = -1   # CCW
        else:
            # ปล่อยปุ่ม → ปลด latch เคลียร์สถานะกด
            self._pressed = False
            self._last_sign = 0
            return

        now = monotonic()

        # ถ้าสลับทิศขณะกดค้าง ให้ถือเป็น "edge ใหม่" ทันที
        if self._pressed and sign != self._last_sign:
            self._pressed = False  # ปลด latch เพื่อให้ยิงครั้งใหม่
        #ถ้ากำลังกดค้างแล้วเปลี่ยนทิศ ถือเป็น “edge ใหม่” เพื่อให้ยิงครั้งใหม่ทันที (ไม่ต้องรอปล่อยจริง)

        # ถ้ายังไม่ได้กด (edge: 0 -> non-zero) ให้ยิงหนึ่งครั้ง + debounce
        if not self._pressed:
            if (now - self._last_click_ts) < DEBOUNCE_S:
                return  # เร็วไป โดน debounce

            # จำนวน pulses ต่อคลิก (ใช้ y ถ้า >0 ไม่งั้นค่า default)
            y = float(msg.linear.y)
            pulses = int(round(y)) if y > 0.0 else deg_to_pulses(DEG_PER_CLICK)

            # เตรียมแพ็กเก็ต one-shot ไป ESP32
            out = Twist()
            out.linear.x = 1.0 if sign > 0 else -1.0  # +1=CW, -1=CCW
            out.linear.y = float(pulses)              # pulses ต่อคลิก
            self._pending_msg = out                   # ให้ timer ส่งครั้งเดียว

            # อัปเดตสถานะสำหรับ monitor/log
            self.last_dir_cw = (sign > 0)
            self.last_pulses = pulses
            self.last_cmd_ts = time()

            self._pressed = True
            self._last_sign = sign
            self._last_click_ts = now

            self.get_logger().info(f"[CLICK] dir={'CW' if sign>0 else 'CCW'}, pulses={pulses}")

        # ถ้ากดค้างและทิศเดิม → ไม่ทำอะไร (ไม่ยิงซ้ำ)
        # ถ้าอยากให้กดค้างแล้วยิงซ้ำ เหมือนคีย์บอร์ด ให้เพิ่ม auto-repeat ตรงนี้ได้

    # ---------- ส่งคำสั่ง (one-shot ผ่าน timer) ----------
    def sendData(self):
        if self._pending_msg is not None:
            self.send_robot_stepper.publish(self._pending_msg)
            self.get_logger().info("[→ /galum/stepper/angle] sent")
            self._pending_msg = None  # เคลียร์คิว ไม่ยิงซ้ำ

def main(args=None):
    rclpy.init()
    sub = stepper()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    
#linear.x = +1.0 (CW) หรือ -1.0 (CCW)
#linear.y = จำนวนพัลส์แบบ one-shot (เช่น 444 เมื่อ 50°/คลิก ที่ 16×)
#linear.y ไปสร้างจำนวนพัลส์จริง (เช่น ยิง 444 พัลส์แล้วหยุด)และเปลี่ยนทิศตาม linear.x (+/-) โดยมี DIR setup time เล็กน้อยก่อนเริ่มยิงพัลส์ (เช่น 5–20 µs)