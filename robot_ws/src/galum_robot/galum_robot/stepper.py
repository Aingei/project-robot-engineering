#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from time import sleep_us
from geometry_msgs.msg import Twist
from rclpy import qos

SPIN_SPEED = 800.0     # steps/s เวลาหมุน

class Stepper(Node):
    def __init__(self):
        super().__init__("stepper")
        
        self.is_spinning = False

        self.send_robot_stepper = self.create_publisher(
            Twist, "/galum/stepper/angle", qos_profile=qos.qos_profile_system_default
        )
        
        self.create_subscription(
            Twist, '/galum/stepper', self.on_toggle, qos_profile=qos.qos_profile_system_default
        )
        
        # timer เรียกทุก 0.5 วิ → หมุน Stepper
        # self.create_timer(0.5, self.auto_rotate)

        # self.sent_data_timer = self.create_timer(0.01, self.sendData) 
        
        self.get_logger().info(f"StepperToggle ready: listening on {INPUT_TOPIC}")
        
     def on_toggle(self, msg: Bool):
        # ถ้า msg.data == True → toggle
        if msg.data:
            self.is_spinning = not self.is_spinning
            self.publish_cmd()
            
    def publish_cmd(self):
        cmd = Twist()
        cmd.linear.x = SPIN_SPEED if self.is_spinning else 0.0
        self.cmd_pub.publish(cmd)
        self.get_logger().info(
            f"Stepper {'ON' if self.is_spinning else 'OFF'} (cmd {cmd.linear.x} sps)"
        )

def main(args=None):
    rclpy.init(args=args)
    node = StepperToggle()
    try:
        rclpy.spin(node)
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()