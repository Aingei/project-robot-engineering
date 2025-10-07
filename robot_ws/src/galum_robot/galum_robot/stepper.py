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
DEG_PER_CLICK = 71            # อยากให้ 50° ต่อคลิก (ปรับตามชอบ) (เดี๋ยวไปคำนวณเป็นพัลส์จริงด้วย deg_to_pulses)

# HOLD_BTN_CW  = 5.0
# HOLD_BTN_CCW = 6.0
DEFAULT_HOLD_TARGET_SPS = 800.0  # ใช้ถ้าจอยไม่ได้ใส่ angular.x มา


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
        
        # NEW: สถานะ “กดค้าง”
        self._hold_active: bool = False
        self._hold_sign: int = 0           # -1/ +1
        self._last_hold_ts: float = 0.0

        # ให้ timer เรียกส่ง "เฉพาะเมื่อมีคำสั่งรอ"
        self.create_timer(0.01, self.sendData)

    # ---------- รับคำสั่งจากจอย ----------
    def on_cmd(self, msg: Twist):
        code = float(msg.linear.x)
        now  = monotonic()

        # ---------- ไม่ใช่ 3/4/5/6 = ปล่อยทุกปุ่ม ----------
        if code not in (3.0, 4.0, 5.0, 6.0):
            if self._hold_active:
                stop = Twist()
                stop.linear.x = 0.0
                stop.linear.y = 0.0
                self._pending_msg = stop
                self._hold_active = False
                self._hold_sign   = 0
                self.get_logger().info("[HOLD-STOP]")
            self._pressed = False
            self._last_sign = 0
            return

        # ---------- โหมดกดค้าง: 5=CW, 6=CCW ----------
        if code in (5.0, 6.0):
            sign = +1 if code == 5.0 else -1

            # ถ้าเปลี่ยนทิศระหว่างค้าง ให้ถือเป็น edge ใหม่
            if self._hold_active and sign != self._hold_sign:
                self._hold_active = False

            if not self._hold_active:
                # debounce hold
                if (now - self._last_hold_ts) < DEBOUNCE_S:
                    return

                target_sps = float(msg.angular.x) if msg.angular.x > 0.0 else DEFAULT_HOLD_TARGET_SPS

                out = Twist()
                out.linear.x = 1.0 if sign > 0 else -1.0   # ทิศ
                out.linear.y = 0.0                         # y=0 = โหมดต่อเนื่อง
                out.angular.x = target_sps                 # steps/s เป้าหมาย
                self._pending_msg = out

                self._hold_active = True
                self._hold_sign   = sign
                self._last_hold_ts = now

                self.last_dir_cw = (sign > 0)
                self.last_pulses = 0
                self.last_cmd_ts = time()

                self.get_logger().info(f"[HOLD-START] dir={'CW' if sign>0 else 'CCW'}, target_sps={target_sps}")
            return  # กำลังค้างอยู่แล้วทิศเดิม → ไม่ยิงซ้ำ

        # ---------- โหมดคลิกทีละก้าว: 3=CW, 4=CCW ----------
        # ถ้ากำลังค้างอยู่แล้วกดคลิก → ส่ง STOP ก่อนเพื่อปิดโหมดต่อเนื่อง
        if self._hold_active:
            stop = Twist()
            stop.linear.x = 0.0
            stop.linear.y = 0.0
            self._pending_msg = stop
            self._hold_active = False
            self._hold_sign   = 0
            self.get_logger().info("[HOLD-TO-CLICK] stop first")

        sign = +1 if code == 3.0 else -1

        # สลับทิศระหว่างคลิกค้าง → edge ใหม่
        if self._pressed and sign != self._last_sign:
            self._pressed = False

        if not self._pressed:
            if (now - self._last_click_ts) < DEBOUNCE_S:
                return  # กันเด้ง

            y = float(msg.linear.y)
            pulses = int(round(y)) if y > 0.0 else deg_to_pulses(DEG_PER_CLICK)

            out = Twist()
            out.linear.x = 1.0 if sign > 0 else -1.0
            out.linear.y = float(pulses)
            self._pending_msg = out

            self.last_dir_cw = (sign > 0)
            self.last_pulses = pulses
            self.last_cmd_ts = time()

            self._pressed = True
            self._last_sign = sign
            self._last_click_ts = now

            self.get_logger().info(f"[CLICK] dir={'CW' if sign>0 else 'CCW'}, pulses={pulses}")

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