#!/usr/bin/env python3
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from rclpy import qos

import time 
import math

class servoload(Node):
    
    def __init__(self):
        super().__init__("servoload")
        
        
        self.servo_angle : float = 0
        
        self.previous_manual_turn = time.time()

        self.send_robot_servo_load = self.create_publisher(
            Twist, "/galum/servo/angle_load", qos_profile=qos.qos_profile_system_default
        )
        
        self.create_subscription(
            Twist, '/galum/servo_load', self.rotate_servo, qos_profile=qos.qos_profile_system_default
        )
        
        # timer เรียกทุก 0.5 วิ → หมุน Stepper
        # self.create_timer(0.5, self.auto_rotate)
        self.sent_data_timer = self.create_timer(0.1, self.sendData)  #adjust slow 0.1
        
    # ---------- ฟังก์ชันหมุน stepper ----------
     
    def rotate_servo(self, msg):
        
        if msg.linear.x == 8.0:               # Closed Stepper
            self.servo_angle =  self.servo_angle + 1

        elif msg.linear.x == 9.0:               # Opened Stepper
            self.servo_angle = self.servo_angle - 1
        
            
        
    def sendData(self):
        # now = time.time()
        # dt = now - self.previous_time
        # self.previous_time = now
        
        # SPEED = 20.0 # degrees per second
        
        # diff = self.servo_angle - self.current_angle
        # step = SPEED * dt
        
        # if abs(diff) <= step:
        #     self.current_angle = self.servo_angle
        # else:
        #     self.current_angle += step if diff > 0 else -step
        
        # Step current_angle by +/- step_deg toward target once per second
        # if self.current_angle < self.servo_angle:
        #     self.current_angle += 1.0
        # elif self.current_angle > self.servo_angle:
        #     self.current_angle -= 1.0

        # limit range
        self.servo_angle = min(170,max(0,self.servo_angle))
        
        servoload_msg = Twist()
        servoload_msg.linear.x = float(self.servo_angle)    
        self.send_robot_servo_load.publish(servoload_msg)
         
def main(args=None):
    rclpy.init()
    sub = servoload()   
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()