#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy import qos
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import time

# ==========================================
# 1. CONFIGURATION (ตั้งค่าตรงนี้ถ้าจะแก้ไข)
# ==========================================
# ตั้งค่า Stepper
AUTO_STEPPER_DIR = 1.0        # 1.0 = ตามเข็ม (CW), -1.0 = ทวนเข็ม (CCW)
AUTO_STEPPER_PULSES = 800.0   # จำนวนพัลส์ที่ต้องการให้หมุน (เช่น 800 พัลส์)

# ตั้งค่ามุม Servo ทั้ง 3 ตัว (อิสระจากกัน)
SERVO1_TARGET_ANGLE = 0.0     # มุมเป้าหมาย Servo 1
SERVO2_TARGET_ANGLE = 45.0    # มุมเป้าหมาย Servo 2
SERVO3_TARGET_ANGLE = 180.0   # มุมเป้าหมาย Servo 3

# ตั้งค่าเวลาหน่วง (วินาที) เพื่อให้ฮาร์ดแวร์ทำงานเสร็จก่อนไป State ถัดไป
DELAY_START = 2.0             # รอ 2 วินาทีก่อนเริ่มทำงาน
DELAY_STEPPER = 3.0           # ให้เวลา Stepper หมุน 3 วินาที
DELAY_SERVO = 1.5             # ให้เวลา Servo แต่ละตัวกาง 1.5 วินาที

# ==========================================
# 2. STATES DEFINITION (กำหนดชื่อ State)
# ==========================================
STATE_INIT         = 0  # รอเวลาเริ่มต้น
STATE_STEPPER_MOVE = 1  # สั่ง Stepper หมุน
STATE_SERVO1_MOVE  = 2  # สั่ง Servo 1
STATE_SERVO2_MOVE  = 3  # สั่ง Servo 2
STATE_SERVO3_MOVE  = 4  # สั่ง Servo 3
STATE_DONE         = 99 # ทำงานเสร็จสิ้น

class AutoRobotController(Node):
    def __init__(self):
        super().__init__("auto_robot_controller")

        # --- Publishers (ส่งคำสั่งไป ESP32) ---
        self.pub_servo = self.create_publisher(Twist, "/galum/servo/angle", qos_profile=qos.qos_profile_system_default)
        self.pub_stepper = self.create_publisher(Twist, "/galum/stepper/angle", qos_profile=qos.qos_profile_system_default)

        # --- Subscribers (รับค่า Ultrasonic เผื่อไว้ใช้เช็คเงื่อนไขอนาคต) ---
        self.sub_ultrasonic = self.create_subscription(Float32MultiArray, "/galum/us_dual", self.on_us_data, qos_profile=qos.qos_profile_system_default)

        # --- ตัวแปรสำหรับคุม Servo (กางแบบนุ่มนวล) ---
        self.target_angles = [90.0, 90.0, 90.0]  # มุมเป้าหมาย
        self.current_angles = [90.0, 90.0, 90.0] # มุมปัจจุบัน (เริ่มที่ 90)
        self.create_timer(0.05, self.update_servos) # อัปเดต Servo ทุกๆ 0.05 วิ

        # --- ตัวแปร State Machine ---
        self.state = STATE_INIT
        self.state_start_time = time.monotonic() # จดเวลาที่เริ่มเข้า State ปัจจุบัน
        
        # Loop หลักของ State Machine ทำงานทุกๆ 0.1 วินาที
        self.create_timer(0.1, self.control_loop)
        
        self.get_logger().info("Auto Robot Started! Waiting to init...")

    # ฟังก์ชันสำหรับเปลี่ยน State และจดเวลาใหม่
    def transition_to(self, new_state, message):
        self.state = new_state
        self.state_start_time = time.monotonic()
        self.get_logger().info(f"Transition -> {message}")

    # ==========================================
    # 3. MAIN STATE MACHINE LOOP
    # ==========================================
    def control_loop(self):
        now = time.monotonic()
        elapsed = now - self.state_start_time # เวลาที่ผ่านไปตั้งแต่เข้า State นี้

        # ------------------------------------------
        # STATE 0: รอให้ระบบเซ็ตอัพเสร็จ
        # ------------------------------------------
        if self.state == STATE_INIT:
            if elapsed > DELAY_START:
                self.transition_to(STATE_STEPPER_MOVE, "STEPPER MOVE")

        # ------------------------------------------
        # STATE 1: สั่ง Stepper ทำงาน
        # ------------------------------------------
        elif self.state == STATE_STEPPER_MOVE:
            if elapsed < 0.2: # สั่งงานแค่ช่วงเสี้ยววินาทีแรก
                out = Twist()
                out.linear.x = AUTO_STEPPER_DIR
                out.linear.y = AUTO_STEPPER_PULSES
                self.pub_stepper.publish(out)
            
            # รอจนกว่าเวลาจะผ่านไปเท่ากับ DELAY_STEPPER (เผื่อเวลาให้มอเตอร์หมุนเสร็จ)
            if elapsed > DELAY_STEPPER:
                self.transition_to(STATE_SERVO1_MOVE, "SERVO 1 MOVE")

        # ------------------------------------------
        # STATE 2: สั่ง Servo ตัวที่ 1
        # ------------------------------------------
        elif self.state == STATE_SERVO1_MOVE:
            # กำหนดมุมเป้าหมาย ตัวอัปเดต Servo จะดึงไปกางเองอย่างนุ่มนวล
            self.target_angles[0] = SERVO1_TARGET_ANGLE
            
            if elapsed > DELAY_SERVO:
                self.transition_to(STATE_SERVO2_MOVE, "SERVO 2 MOVE")

        # ------------------------------------------
        # STATE 3: สั่ง Servo ตัวที่ 2
        # ------------------------------------------
        elif self.state == STATE_SERVO2_MOVE:
            self.target_angles[1] = SERVO2_TARGET_ANGLE
            
            if elapsed > DELAY_SERVO:
                self.transition_to(STATE_SERVO3_MOVE, "SERVO 3 MOVE")

        # ------------------------------------------
        # STATE 4: สั่ง Servo ตัวที่ 3
        # ------------------------------------------
        elif self.state == STATE_SERVO3_MOVE:
            self.target_angles[2] = SERVO3_TARGET_ANGLE
            
            if elapsed > DELAY_SERVO:
                self.transition_to(STATE_DONE, "ALL TASKS DONE")

        # ------------------------------------------
        # STATE 99: จบการทำงาน
        # ------------------------------------------
        elif self.state == STATE_DONE:
            # หุ่นยนต์จะหยุดนิ่งใน State นี้
            # หากต้องการให้วนลูปกลับไปทำใหม่เรื่อยๆ ให้เปิดคอมเมนต์บรรทัดล่าง:
            # self.transition_to(STATE_INIT, "RESTARTING LOOP")
            pass

    # ==========================================
    # 4. BACKGROUND SERVO SWEEP
    # ==========================================
    def update_servos(self):
        changed = False
        for i in range(3):
            if self.current_angles[i] < self.target_angles[i]:
                self.current_angles[i] += 1.0
                changed = True
            elif self.current_angles[i] > self.target_angles[i]:
                self.current_angles[i] -= 1.0
                changed = True
            
            self.current_angles[i] = max(0.0, min(180.0, self.current_angles[i]))
            
        if changed:
            out = Twist()
            out.linear.x = float(self.current_angles[0])
            out.linear.y = float(self.current_angles[1])
            out.linear.z = float(self.current_angles[2])
            self.pub_servo.publish(out)

    def on_us_data(self, msg: Float32MultiArray):
        # เผื่ออนาคตต้องการเอาเซนเซอร์ Ultrasonic มาเป็นเงื่อนไขในการเปลี่ยน State (เช่น เจอกำแพงแล้วค่อยสั่ง Stepper)
        pass

def main(args=None):
    rclpy.init(args=args)
    node = AutoRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()