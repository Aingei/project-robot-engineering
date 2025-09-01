#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from time import sleep_us

# GPIO abstraction สำหรับ Pi หรือ ESP32
try:
    from machine import Pin   # ESP32
except ImportError:
    class Pin:  # Mock GPIO สำหรับทดสอบบนคอม
        OUT = 0
        def __init__(self, pin, mode): pass
        def value(self, val=None): pass

# กำหนดขา
DIR_PIN = Pin(26, Pin.OUT)
STEP_PIN = Pin(25, Pin.OUT)
DELAY = 800  # ความเร็ว step (us)

running = False  # สถานะหมุน

def step_motor_continuous(direction=1):
    DIR_PIN.value(direction)
    while running:
        STEP_PIN.value(1)
        sleep_us(DELAY)
        STEP_PIN.value(0)
        sleep_us(DELAY)

class StepperSimple(Node):
    def __init__(self):
        super().__init__("stepper_simple")

        # Subscriber รับคำสั่งหมุน True=หมุน, False=หยุด
        self.create_subscription(
            Bool, "/cmd_step", self.cmd_callback, 10
        )

    def cmd_callback(self, msg: Bool):
        global running
        running = msg.data
        if running:
            self.get_logger().info("Motor START")
        else:
            self.get_logger().info("Motor STOP")

def main():
    rclpy.init()
    node = StepperSimple()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        global running
        running = False
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
