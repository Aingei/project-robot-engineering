#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy import qos
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
import time

# ==========================================
# 1. CONFIGURATION (ตั้งค่าการเคลื่อนที่)
# ==========================================
# 1.1 Stepper Motor
STEPPER_DOWN_DIR = 1.0        # ทิศทางลงไปที่ดิน (สมมติ 1.0 คือลง)
STEPPER_DOWN_PULSES = 2000.0  # จำนวนพัลส์ให้ลงจนสุด (ปรับตัวเลขตามจริง)

STEPPER_UP_DIR = -1.0         # ทิศทางยกขึ้น (ตรงข้ามกับตอนลง)
STEPPER_UP_PULSES = 500.0     # จำนวนพัลส์ที่ยกขึ้นมานิดนึง

# 1.2 Servo Motors (องศา)
S1_STORE_ANGLE = 90.0         # Servo 1: เก็บแขน
S2_CLOSE_ANGLE = 90.0         # Servo 2: หมุนปิด
S3_DROP_ANGLE  = 180.0        # Servo 3: เปิดปล่อยต้นกล้า

# 1.3 Servo 4 (360 องศา สำหรับเจาะดิน)
S4_DRILL_SPEED = 180.0        # 180 = หมุนเจาะ, 90 = หยุดเจาะ

# 1.4 เวลาหน่วง (Delay) ของแต่ละขั้นตอน (วินาที)
DELAY_START = 2.0             # รอระบบพร้อมก่อนเริ่มทำงาน
DELAY_STEPPER_DOWN = 5.0      # รอ Stepper วิ่งลงสุด (สมมติใช้เวลา 5 วิ)
DELAY_SERVO_ARM = 1.5         # รอ Servo 1 และ 2 ขยับเสร็จ
DELAY_DRILL_TIME = 3.0        # *** เจาะดินนาน 3 วินาที ***
DELAY_STEPPER_UP = 2.0        # รอ Stepper ยกขึ้นนิดนึง (สมมติใช้เวลา 2 วิ)
DELAY_SERVO_DROP = 1.5        # รอ Servo 3 เปิดสุด

# ==========================================
# 2. STATES DEFINITION (ลำดับการทำงาน)
# ==========================================
STATE_INIT         = 0  
STATE_STEPPER_DOWN = 1  # Stepper ลงสุด
STATE_FOLD_AND_CLOSE = 2  # S1 เก็บแขน, S2 ปิด
STATE_DRILL_SOIL   = 3  # S4 เจาะดิน (3 วินาที)
STATE_STEPPER_UP   = 4  # หยุดเจาะ + Stepper ยกขึ้นนิดนึง
STATE_DROP_SEED    = 5  # S3 เปิดปล่อยต้นกล้า
STATE_DONE         = 99 

class PlanterRobotController(Node):
    def __init__(self):
        super().__init__("planter_robot_controller")

        self.pub_servo = self.create_publisher(Twist, "/galum/servo/angle", qos_profile=qos.qos_profile_system_default)
        self.pub_stepper = self.create_publisher(Twist, "/galum/stepper/angle", qos_profile=qos.qos_profile_system_default)
        self.sub_ultrasonic = self.create_subscription(Float32MultiArray, "/galum/us_dual", self.on_us_data, qos_profile=qos.qos_profile_system_default)

        # มุมเริ่มต้น [S1, S2, S3, S4(360)] -> 0 คือเริ่มที่องศา 0, ตัวที่ 4 เป็น 90 คือหยุดหมุน
        self.target_angles = [0.0, 0.0, 0.0, 90.0]  
        self.current_angles = [0.0, 0.0, 0.0, 90.0] 
        self.create_timer(0.05, self.update_servos)

        self.state = STATE_INIT
        self.state_start_time = time.monotonic() 
        
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Planter Robot Started! Waiting to init...")

    def transition_to(self, new_state, message):
        self.state = new_state
        self.state_start_time = time.monotonic()
        self.get_logger().info(f"Transition -> {message}")

    # ==========================================
    # 3. MAIN SEQUENCE LOOP
    # ==========================================
    def control_loop(self):
        now = time.monotonic()
        elapsed = now - self.state_start_time 

        # ------------------------------------------
        # STATE 0: พร้อมเริ่มงาน
        # ------------------------------------------
        if self.state == STATE_INIT:
            if elapsed > DELAY_START:
                self.transition_to(STATE_STEPPER_DOWN, "1. STEPPER DOWN (Lack)")

        # ------------------------------------------
        # STATE 1: Stepper ดันลงสุด
        # ------------------------------------------
        elif self.state == STATE_STEPPER_DOWN:
            if elapsed < 0.2: 
                out = Twist()
                out.linear.x = STEPPER_DOWN_DIR
                out.linear.y = STEPPER_DOWN_PULSES
                self.pub_stepper.publish(out)
            
            if elapsed > DELAY_STEPPER_DOWN:
                self.transition_to(STATE_FOLD_AND_CLOSE, "2. SERVO 1 FOLD & SERVO 2 CLOSE")

        # ------------------------------------------
        # STATE 2: Servo 1 เก็บแขน + Servo 2 ปิด
        # ------------------------------------------
        elif self.state == STATE_FOLD_AND_CLOSE:
            self.target_angles[0] = S1_STORE_ANGLE
            self.target_angles[1] = S2_CLOSE_ANGLE
            
            if elapsed > DELAY_SERVO_ARM:
                self.transition_to(STATE_DRILL_SOIL, "3. DRILLING SOIL (3 SECONDS)")

        # ------------------------------------------
        # STATE 3: Servo 4 (360) เปิดเจาะดิน 3 วิ
        # ------------------------------------------
        elif self.state == STATE_DRILL_SOIL:
            if elapsed < DELAY_DRILL_TIME:
                self.target_angles[3] = S4_DRILL_SPEED # สั่งหมุนเจาะ
            else:
                self.target_angles[3] = 90.0 # ครบ 3 วิ -> สั่งหยุดเจาะ
                self.transition_to(STATE_STEPPER_UP, "4. STEPPER UP A LITTLE BIT")

        # ------------------------------------------
        # STATE 4: Stepper ยกขึ้นมานิดนึง
        # ------------------------------------------
        elif self.state == STATE_STEPPER_UP:
            if elapsed < 0.2:
                out = Twist()
                out.linear.x = STEPPER_UP_DIR       # ทิศทางขึ้น
                out.linear.y = STEPPER_UP_PULSES    # จำนวนพัลส์นิดหน่อย
                self.pub_stepper.publish(out)
            
            if elapsed > DELAY_STEPPER_UP:
                self.transition_to(STATE_DROP_SEED, "5. SERVO 3 OPEN (DROP SEEDLING)")

        # ------------------------------------------
        # STATE 5: Servo 3 เปิดปล่อยต้นกล้า
        # ------------------------------------------
        elif self.state == STATE_DROP_SEED:
            self.target_angles[2] = S3_DROP_ANGLE
            
            if elapsed > DELAY_SERVO_DROP:
                self.transition_to(STATE_DONE, "PLANTING SEQUENCE COMPLETED!")

        # ------------------------------------------
        # STATE 99: จบการทำงาน
        # ------------------------------------------
        elif self.state == STATE_DONE:
            pass

    # ==========================================
    # 4. BACKGROUND SERVO SWEEP
    # ==========================================
    def update_servos(self):
        changed = False
        
        # จัดการ Servo 180 องศา (Index 0, 1, 2) -> ให้ค่อยๆ กวาดมุมนุ่มนวล
        for i in [0, 1, 2]:
            if self.current_angles[i] < self.target_angles[i]:
                self.current_angles[i] += 2.0 # ขยับทีละ 2 องศา
                changed = True
            elif self.current_angles[i] > self.target_angles[i]:
                self.current_angles[i] -= 2.0
                changed = True
            self.current_angles[i] = max(0.0, min(180.0, self.current_angles[i]))
            
        # จัดการ Servo 360 องศา (Index 3) -> ส่งค่าตรงๆ ไม่ต้องค่อยๆ กวาด
        if self.current_angles[3] != self.target_angles[3]:
            self.current_angles[3] = self.target_angles[3]
            changed = True
            
        if changed:
            out = Twist()
            out.linear.x = float(self.current_angles[0])  # Servo 1
            out.linear.y = float(self.current_angles[1])  # Servo 2
            out.linear.z = float(self.current_angles[2])  # Servo 3
            out.angular.x = float(self.current_angles[3]) # Servo 4 (360 เจาะดิน)
            self.pub_servo.publish(out)

    def on_us_data(self, msg: Float32MultiArray):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = PlanterRobotController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()