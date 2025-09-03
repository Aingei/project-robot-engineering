#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from time import sleep_us
from geometry_msgs.msg import Twist
from rclpy import qos

STEPS_PER_REV = 200
# MICROSTEP = 16
DELAY = 0.001 

class StepperSimple(Node):
    def __init__(self):
        super().__init__("stepper_simple")
        
        self.target_angle = 90.0
        self.current_steps = 0
        self.current_angle = 0.0

        self.send_robot_stepper = self.create_publisher(
            Twist, "/galum/stepper/angle", qos_profile=qos.qos_profile_system_default
        )
        
        self.create_subscription(
            Twist, '/galum/stepper', self.rotate_stepper, qos_profile=qos.qos_profile_system_default
        )
        
        # timer เรียกทุก 0.5 วิ → หมุน Stepper
        self.create_timer(0.5, self.auto_rotate)

        self.sent_data_timer = self.create_timer(0.01, self.sendData) 
        
    # ---------- ฟังก์ชันหมุน stepper ----------
    def rotate_stepper(self, target_angle):
        target_steps = int((target_angle / 360.0) * STEPS_PER_REV * MICROSTEP)
        steps_to_move = target_steps - self.current_steps

        direction = "CW" if steps_to_move >= 0 else "CCW"
        print(f"[SIM] Rotate {direction}: {abs(steps_to_move)} steps")

        # simulate delay
        time.sleep(abs(steps_to_move) * DELAY)

        self.current_steps = target_steps
        self.current_angle = (self.current_steps * 360.0) / (STEPS_PER_REV * MICROSTEP)
    
    def auto_rotate(self):
        self.rotate_stepper(self.target_angle)
        
    def stepper_angle(self, msg: Twist):
        self.target_angle = msg.linear.x

    def sendData(self):
        msg = Twist()
        msg.linear.x = self.current_angle
        self.send_robot_stepper.publish(msg)
        self.get_logger().debug(f"Current angle: {self.current_angle} deg")

def main(args=None):
    rclpy.init(args=args)
    node = StepperSimple()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()