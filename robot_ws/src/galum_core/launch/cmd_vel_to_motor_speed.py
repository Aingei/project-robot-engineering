#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Twist

from rclpy import qos
from galum_core.utilize import * 
from galum_core.controller import *
import time 
import math


class Cmd_vel_to_motor_speed(Node):


    moveSpeed: float = 0.0
    slideSpeed: float = 0.0
    turnSpeed: float = 0.0

    # maxSpeed : int = 1023.0/2 # pwm
    # max_linear_speed = 2.0  # m/s max
    motor1Speed : float = 0
    motor2Speed : float = 0
    motor3Speed : float = 0
    motor4Speed : float = 0


    def __init__(self):
        super().__init__("Cmd_vel_to_motor_speed")
        
        # self.declare_parameter("motor1", True)
        # self.declare_parameter("motor2", True)
        # self.declare_parameter("motor3", True)
        # self.declare_parameter("motor4", True)
        
        # self.motor1_enabled = self.get_parameter("motor1").get_parameter_value().bool_value
        # self.motor2_enabled = self.get_parameter("motor2").get_parameter_value().bool_value
        # self.motor3_enabled = self.get_parameter("motor3").get_parameter_value().bool_value
        # self.motor4_enabled = self.get_parameter("motor4").get_parameter_value().bool_value

        
        self.moveSpeed: float = 0.0
        self.slideSpeed: float = 0.0
        self.turnSpeed: float = 0.0

        self.maxSpeed : float = 1023.0 # pwm
        self.maxRPM : int = 3000
        self.max_linear_speed = 3.0  # m/s max
        self.motor1Speed : float = 0
        self.motor2Speed : float = 0
        self.motor3Speed : float = 0
        self.motor4Speed : float = 0

        self.motorshooter1Speed : float = 0
        self.motorshooter2Speed : float = 0
        self.motorshooter3Speed : float = 0
        self.yaw : float = 0
        self.yaw_setpoint = self.yaw
        
        self.macro_active = False
        self.previous_manual_turn = time.time()

        self.controller = Controller()
        
 
        
        self.send_robot_speed = self.create_publisher(
            Twist, "/galum/cmd_move/rpm", qos_profile=qos.qos_profile_system_default
        )

        self.send_shoot_speed = self.create_publisher(
            Twist, "/galum/cmd_gripper_dig/rpm", qos_profile=qos.qos_profile_system_default
        )


        self.create_subscription(
            Twist, '/galum/cmd_move', self.cmd_vel, qos_profile=qos.qos_profile_system_default
        )

        self.create_subscription(
            Twist, '/galum/cmd_gripper_dig', self.cmd_gripper, qos_profile=qos.qos_profile_sensor_data # 10
        )
      

        self.create_subscription(
            Twist, '/galum/imu/pos_angle', self.get_robot_angle, qos_profile=qos.qos_profile_sensor_data # 10
        )

        self.create_subscription(
            Float32MultiArray, '/galum/pid/rotate', self.get_pid, qos_profile=qos.qos_profile_sensor_data # 10
        )


        self.sent_data_timer = self.create_timer(0.03, self.sendData)
        
    def get_robot_angle(self,msg):
        self.yaw = WrapRads(To_Radians(msg.angular.z))
    
    def get_pid(self,msg):
        self.controller.ConfigPIDF(kp = msg.data[0], ki= msg.data[1], kd=msg.data[2], kf=msg.data[3]) 


    #Diff Drive 4 Wheels
    def cmd_vel(self, msg):
        # รับคำสั่งความเร็ว
        self.moveSpeed = msg.linear.x    # เดินหน้า/ถอยหลัง
        self.slideSpeed = 0              # ไม่มีเลื่อนซ้าย-ขวาใน diff drive
        self.turnSpeed = msg.angular.z   # หมุนรอบแกน z
        
        rotation = self.turnSpeed

        # คำนวณความเร็วล้อซ้ายและขวา
        left_speed = self.moveSpeed - (self.turnSpeed / 2.0) * rotation
        right_speed = self.moveSpeed + (self.turnSpeed / 2.0) * rotation

        # ปรับสเกลความเร็ว (normalize) ถ้าจำเป็น
        max_wheel_speed = max(abs(left_speed), abs(right_speed), 1.0)
        left_speed = left_speed / max_wheel_speed * self.maxSpeed
        right_speed = right_speed / max_wheel_speed * self.maxSpeed

        # กำหนดความเร็วล้อ 4 ล้อ (ล้อซ้าย 2 ล้อ, ล้อขวา 2 ล้อ)
        self.motor1Speed = float("{:.1f}".format(left_speed))   # ล้อซ้ายหน้า
        self.motor2Speed = float("{:.1f}".format(left_speed))   # ล้อซ้ายหลัง
        self.motor3Speed = float("{:.1f}".format(right_speed))  # ล้อขวาหน้า
        self.motor4Speed = float("{:.1f}".format(right_speed))  # ล้อขวาหลัง

            
    def sendData(self):
        motorspeed_msg = Twist()
       
        motorspeed_msg.linear.x = float(self.motor1Speed)
        motorspeed_msg.linear.y = float(self.motor2Speed)
        motorspeed_msg.angular.x = float(self.motor3Speed)
        motorspeed_msg.angular.y = float(self.motor4Speed)

        
        self.send_robot_speed.publish(motorspeed_msg)


def main():
    rclpy.init()

    sub = Cmd_vel_to_motor_speed()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()