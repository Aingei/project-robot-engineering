#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

# ─────────────────────────────────────────────────────
#  States
# ─────────────────────────────────────────────────────
STATE_INIT               = 0
STATE_SERVO4_ON          = 1
STATE_STEPPER_DOWN_800   = 2
STATE_STEPPER_UP_800     = 3
STATE_SERVO4_OFF         = 4
STATE_SERVO4_SWAP        = 12
STATE_STEPPER_DOWN_400   = 5
STATE_SERVO3_0           = 6
STATE_SERVO3_100         = 7
STATE_STEPPER_UP_400     = 8
STATE_SERVO2_180         = 9
STATE_SERVO2_0           = 10
STATE_DONE               = 11

class RobotSequenceNode(Node):
    def __init__(self):
        super().__init__('robot_sequence_node')

        # ── Publishers ──────────────────────────────────────
        self.servo_pub = self.create_publisher(Twist, '/galum/servo/angle', 10)
        self.stepper_pub = self.create_publisher(Twist, '/galum/stepper/angle', 10)
        
        # ── State Variables ─────────────────────────────────
        self.state = STATE_INIT
        self.state_start_time = time.monotonic()
        self.state_wait_time = 2.0  # เวลารอเริ่มต้น
        self.action_triggered = False
        
        # ── Servo Default Values ────────────────────────────
        self.current_servo1 = 90.0
        self.current_servo2 = 90.0
        self.current_servo3 = 90.0
        self.current_servo4 = 90.0

        # ใช้ Timer รันลูป State Machine ทุกๆ 0.1 วินาที (10Hz)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("🚀 Start Robot Sequence (State Machine)")

    # ─────────────────────────────────────────────────────
    #  Helper Functions
    # ─────────────────────────────────────────────────────
    def send_servo_cmd(self):
        msg = Twist()
        msg.linear.x = float(self.current_servo1)
        msg.linear.y = float(self.current_servo2)
        msg.linear.z = float(self.current_servo3)
        msg.angular.x = float(self.current_servo4)
        self.servo_pub.publish(msg)

    def send_stepper_cmd(self, direction, pulses):
        msg = Twist()
        msg.linear.x = float(direction)  # -1.0 = ลง (CW), 1.0 = ขึ้น (CCW)
        msg.linear.y = float(pulses)     # จำนวนพัลส์
        self.stepper_pub.publish(msg)

    def change_state(self, new_state, wait_time):
        self.state = new_state
        self.state_start_time = time.monotonic()
        self.state_wait_time = wait_time
        self.action_triggered = False

    # ─────────────────────────────────────────────────────
    #  State Machine Loop
    # ─────────────────────────────────────────────────────
    def control_loop(self):
        # 1. รันคำสั่ง 1 ครั้งเมื่อเพิ่งเข้าสู่ State ใหม่
        if not self.action_triggered:
            self.execute_state_action()
            self.action_triggered = True
            
        # 2. เช็คเวลาว่าครบกำหนด Wait Time ของ State นี้หรือยัง
        if time.monotonic() - self.state_start_time >= self.state_wait_time:
            self.transition_to_next_state()

    def execute_state_action(self):
        if self.state == STATE_INIT:
            self.get_logger().info("[STATE 0] Set Initial Position: Servo 1=90, Others=90")
            self.current_servo1 = 90.0
            self.current_servo2 = 90.0
            self.current_servo3 = 90.0
            self.current_servo4 = 90.0
            self.send_servo_cmd()
            
        elif self.state == STATE_SERVO4_ON:
            self.get_logger().info("[STATE 1] Servo 4 หมุนเปิด (60 deg)")
            self.current_servo4 = 60.0
            self.send_servo_cmd()

        elif self.state == STATE_STEPPER_DOWN_800:
            self.get_logger().info("[STATE 2] Stepper เอาแขนลง (-1.0, 800 pulse)")
            self.send_stepper_cmd(-1.0, 350)
            
        elif self.state == STATE_SERVO4_SWAP:
            self.get_logger().info("SWAP SIDE")
            self.current_servo4 = 120.0
            self.send_servo_cmd()

        elif self.state == STATE_STEPPER_UP_800:
            self.get_logger().info("[STATE 3] Stepper เอาแขนขึ้น (1.0, 800 pulse)")
            self.send_stepper_cmd(1.0, 800)

        elif self.state == STATE_SERVO4_OFF:
            self.get_logger().info("[STATE 4] Servo 4 หมุนกลับหยุด (90 deg)")
            self.current_servo4 = 90.0
            self.send_servo_cmd()

        elif self.state == STATE_STEPPER_DOWN_400:
            self.get_logger().info("[STATE 5] Stepper เอาแขนลง (-1.0, 400 pulse)")
            self.send_stepper_cmd(-1.0, 150)

        elif self.state == STATE_SERVO3_0:
            self.get_logger().info("[STATE 6] Servo 3 กางออก (0 deg)")
            self.current_servo3 = 0.0
            self.send_servo_cmd()

        elif self.state == STATE_SERVO3_100:
            self.get_logger().info("[STATE 7] Servo 3 เก็บแขน (100 deg)")
            self.current_servo3 = 100.0
            self.send_servo_cmd()

        elif self.state == STATE_STEPPER_UP_400:
            self.get_logger().info("[STATE 8] Stepper เอาแขนขึ้น (1.0, 400 pulse)")
            self.send_stepper_cmd(1.0, 800)

        elif self.state == STATE_SERVO2_180:
            self.get_logger().info("[STATE 9] Servo 2 หมุนไปที่ (180 deg)")
            self.current_servo2 = 180.0
            self.send_servo_cmd()

        elif self.state == STATE_SERVO2_0:
            self.get_logger().info("[STATE 10] Servo 2 หมุนไปที่ (0 deg)")
            self.current_servo2 = 0.0
            self.send_servo_cmd()

        elif self.state == STATE_DONE:
            self.get_logger().info("[STATE 11] จบการทำงาน Servo 1 = 90 deg")
            self.current_servo1 = 90.0
            self.current_servo2 = 90.0 # ตัดการทำงานกลับที่เดิมด้วยความปลอดภัย
            self.send_servo_cmd()

    def transition_to_next_state(self):
        # เปลี่ยน State พร้อมกำหนดเวลาที่รอให้ฮาร์ดแวร์ทำงานเสร็จ (ปรับได้ตามความเหมาะสม)
        if self.state == STATE_INIT:
            self.change_state(STATE_SERVO4_ON, wait_time=2.0)
        elif self.state == STATE_SERVO4_ON:
            self.change_state(STATE_STEPPER_DOWN_800, wait_time=4.0) # รอ Stepper 800 pulse
        
        elif self.state == STATE_STEPPER_DOWN_800:
            self.change_state(STATE_SERVO4_SWAP, wait_time=4.0)
        
        elif self.state == STATE_SERVO4_SWAP:
            self.change_state(STATE_STEPPER_UP_800, wait_time=4.0)   # รอ Stepper 800 pulse
        elif self.state == STATE_STEPPER_UP_800:
            self.change_state(STATE_SERVO4_OFF, wait_time=2.0)
        elif self.state == STATE_SERVO4_OFF:
            self.change_state(STATE_STEPPER_DOWN_400, wait_time=3.0) # รอ Stepper 400 pulse
        elif self.state == STATE_STEPPER_DOWN_400:
            self.change_state(STATE_SERVO3_0, wait_time=2.0)
        elif self.state == STATE_SERVO3_0:
            self.change_state(STATE_SERVO3_100, wait_time=2.0)
        elif self.state == STATE_SERVO3_100:
            self.change_state(STATE_STEPPER_UP_400, wait_time=3.0)   # รอ Stepper 400 pulse
        elif self.state == STATE_STEPPER_UP_400:
            self.change_state(STATE_SERVO2_180, wait_time=2.0)
        elif self.state == STATE_SERVO2_180:
            self.change_state(STATE_SERVO2_0, wait_time=2.0)
        elif self.state == STATE_SERVO2_0:
            self.change_state(STATE_DONE, wait_time=99999.0) # ค้างที่ State จบไว้ตลอดไป


def main():
    rclpy.init()
    node = RobotSequenceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("หยุดการทำงาน Sequence")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()