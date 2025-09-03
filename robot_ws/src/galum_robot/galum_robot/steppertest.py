#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from time import sleep_us
from geometry_msgs.msg import Twist
from rclpy import qos

# STEPS_PER_REV = 200
# MICROSTEP = 16
# DELAY = 0.001 

class StepperSimple(Node):
    def __init__(self):
        super().__init__("stepper_simple")
        
        self.stepper_angle = 0.0
        

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
     
    def rotate_stepper(self, msg):
        
        if msg.linear.x == 1:               # Closed Stepper
            self.stepper_angle = float(0.0)

        if msg.linear.x == 2:               # Opened Stepper
            self.stepper_angle = float(90.0)
            
    def sendData(self):
        stepper_msg = Twist()
       
        stepper_msg.linear.x = float(self.stepper_angle)
        
        
        self.send_robot_stepper.publish(stepper_msg)
         
def main(args=None):
    rclpy.init()
    sub = steppertest()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()