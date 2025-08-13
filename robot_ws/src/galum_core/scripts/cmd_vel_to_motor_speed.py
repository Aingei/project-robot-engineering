#!/usr/bin/env python3

# from pynput import keyboard
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist
from rclpy import qos
from galum_core.utilize import *
from galum_core.controller import *
import time 

class Cmd_vel_to_motor_speed(Node):

    def __init__(self):
        super().__init__("Cmd_vel_to_motor_speed")
        
        self.declare_parameter("motor1", True)
        self.declare_parameter("motor2", True)
        self.declare_parameter("motor3", True)
        self.declare_parameter("motor4", True)
        
        self.motor1_enabled = self.get_parameter("motor1").get_parameter_value().bool_value
        self.motor2_enabled = self.get_parameter("motor2").get_parameter_value().bool_value
        self.motor3_enabled = self.get_parameter("motor3").get_parameter_value().bool_value
        self.motor4_enabled = self.get_parameter("motor4").get_parameter_value().bool_value

        
        self.moveSpeed: float = 0.0
        self.turnSpeed: float = 0.0

        self.maxSpeed : float = 1023.0
        self.motor1Speed : float = 0
        self.motor2Speed : float = 0
        
        self.trackWidth : float = 2.0

        self.yaw : float = 0
        self.yaw_setpoint = self.yaw

        self.previous_manual_turn = time.time()

        self.controller = Controller(kp = 1.0, ki = 0.05, kd = 0.001, errorTolerance= To_Radians(0.5), i_min= -1, i_max= 1)
        
        
        # self.macro_active = False
        

        self.send_robot_speed = self.create_publisher(
            Twist, "/galum/cmd_move/rpm", qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Twist, '/galum/cmd_move', self.cmd_move, qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Float32MultiArray, '/galum/pid/rotate', self.get_pid, qos_profile=qos.qos_profile_sensor_data # 10
        )
        
        self.create_subscription(
            Twist, '/galum/imu/pos_angle', self.get_robot_angle, qos_profile=qos.qos_profile_sensor_data # 10
        )


        self.sent_data_timer = self.create_timer(0.01, self.sendData)
        

    def get_robot_angle(self,msg):
        self.yaw = WrapRads(To_Radians(msg.linear.x))

    def get_pid(self,msg):
        self.controller.ConfigPIDF(kp = msg.data[0], ki= msg.data[1], kd=msg.data[2], kf=msg.data[3]) 


    def cmd_move(self, msg):

        
        # Initialize CurrentTime at the start
        CurrentTime = time.time()

        # Handle turning behavior and set the yaw setpoint
        if self.turnSpeed != 0 or (CurrentTime - self.previous_manual_turn < 0.45):
            rotation = self.turnSpeed
            self.yaw_setpoint = self.yaw
        else:
            # Apply PID controller based on yaw setpoint
            rotation = self.controller.Calculate(WrapRads(self.yaw_setpoint - self.yaw))

        # If no movement or turn, stop rotation (ignoring slideSpeed logic)
        if self.moveSpeed == 0 and self.turnSpeed == 0 :
            rotation = 0
            self.yaw_setpoint = self.yaw

        # Update the previous manual turn time
        self.previous_manual_turn = CurrentTime if self.turnSpeed != 0 else self.previous_manual_turn

        # Set move and turn speeds from the message input
        self.moveSpeed = msg.linear.x
        self.turnSpeed = msg.angular.x 

        self.turnSpeed = self.turnSpeed / 3

        # Calculate motor speeds based on move and turn speeds
        self.motor1Speed = (self.moveSpeed + rotation) * self.maxSpeed #Left Start Slower
        self.motor2Speed = (self.moveSpeed - rotation) * self.maxSpeed #Right
        

        if self.motor1Speed >= self.maxSpeed:
            self.motor1Speed = self.maxSpeed

        if self.motor2Speed >= self.maxSpeed:
            self.motor2Speed = self.maxSpeed    
        # self.rotation = rotation * self.maxSpeed  # Apply track width to rotation speed
 
    def sendData(self):
        motorspeed_msg = Twist()
       
        motorspeed_msg.linear.x = float(self.motor1Speed) #Left
        motorspeed_msg.linear.y = float(self.motor2Speed) #Right

        self.send_robot_speed.publish(motorspeed_msg)
       

def main():
    rclpy.init()

    sub = Cmd_vel_to_motor_speed()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()