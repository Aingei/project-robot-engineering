#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from rclpy import qos

import time 
import math

class servo(Node):
    
    def __init__(self):
        super().__init__("servo")
        
        
        self.servo_angle : float = 0
        
        self.previous_manual_turn = time.time()

        self.send_robot_servo = self.create_publisher(
            Twist, "/galum/servo/angle", qos_profile=qos.qos_profile_system_default
        )
        
        self.create_subscription(
            Twist, '/galum/servo', self.rotate_servo, qos_profile=qos.qos_profile_system_default
        )
        
        # timer เรียกทุก 0.5 วิ → หมุน Stepper
        # self.create_timer(0.5, self.auto_rotate)

        self.sent_data_timer = self.create_timer(0.01, self.sendData) 
        
    # ---------- ฟังก์ชันหมุน stepper ----------
     
    def rotate_servo(self, msg):
        
        if msg.linear.x == 1:               # Closed Stepper
            self.servo_angle = float(0.0)

        elif msg.linear.x == 2:               # Opened Stepper
            self.servo_angle = float(90.0)
            
    def sendData(self):
        servo_msg = Twist()
       
        servo_msg.linear.x = float(self.servo_angle)
        
        
        self.send_robot_servo.publish(servo_msg)
         
def main(args=None):
    rclpy.init()
    sub = servo()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()