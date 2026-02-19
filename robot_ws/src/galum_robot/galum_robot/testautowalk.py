#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from galum_robot.utilize import *
from galum_robot.controller import *
# from motors_interfaces.msg import Motor
import math

class AutoWalk(Node):
    def __init__(self):
        super().__init__('autowalk')

        self.send_robot_speed = self.create_publisher(
            Twist, "/galum/cmd_move/rpm", 10 )
        
        self.send_cmd_vel = self.create_publisher(
            Twist, 
            "/galum/cmd_vel_monitor",  # ตั้งชื่อ topic ใหม่สำหรับดู
            10
        )
        
        self.create_subscription(Twist, "/galum/imu_angle", self.get_robot_angle, 10)
        
        self.create_subscription(Twist, "/galum/encoder", self.encoder_callback, 10)
        
        self.timer = self.create_timer(0.05, self.loop)

        # ===== ปรับตรงนี้ =====
        self.moveSpeed = 10.0        # ความเร็ว
        self.target_distance = 1.0  # 1 m
        self.walk_time = 10.0      # เดินกี่วินาที
        # =====================
        
        self.turnSpeed = 0.0

        self.previous_time = time.time()
        self.start_time = time.time()
        self.stopped = False
        
        self.yaw = 0.0
        self.yaw_setpoint = 0.0 #self.yaw_setpoint = self.yaw
        self.maxSpeed : float = 1023.0
        
        self.motor1Speed : float = 0
        self.motor2Speed : float = 0
        self.motor3Speed : float = 0
        self.motor4Speed : float = 0
        
        #Encoder
        
        self.wheel_radius = 0.05        # 50mm แปลงเป็นเมตร
        self.ticks_per_rev = 2640.0       #pulse
        
        circumference = 2 * math.pi * self.wheel_radius
        self.m_per_tick = circumference / self.ticks_per_rev
        
        self.prev_ticks = [0, 0, 0, 0]
        self.total_distance = 0.0
        self.first_run = True   
        
        self.controller = Controller(kp = 1.0, ki = 0.05, kd = 0.001, errorTolerance= To_Radians(0.5), i_min= -1, i_max= 1)
        

        self.get_logger().info('AUTO WALK START')
    
    def encoder_callback(self,msg):
        current_ticks = [
            msg.linear.x,   # FL
            msg.linear.y,   # FR
            msg.angular.x,  # RL
            msg.angular.y   # RR
        ]
        
        if self.first_run:
            self.prev_ticks = current_ticks
            self.first_run = False
            return
        
        distances = []
        
        for i in range(4):
            diff = current_ticks[i] - self.prev_ticks[i] # ผลต่าง Tick
            distance_m = diff * self.m_per_tick              # แปลงเป็นเมตร
            distances.append(distance_m)
            
        avg_left = (distances[0] + distances[2]) / 2.0
        avg_right = (distances[1] + distances[3]) / 2.0
        
        center_distance = (avg_left + avg_right) / 2.0
        
        self.total_distance += center_distance
        
        self.prev_ticks = current_ticks
            
            
    def get_robot_angle(self,msg):
        self.yaw = WrapRads(To_Radians(msg.linear.x))

    def get_pid(self,msg):
        self.controller.ConfigPIDF(kp = msg.data[0], ki= msg.data[1], kd=msg.data[2], kf=msg.data[3]) 

    def loop(self):
        
        msg = Twist()
        elapsed = time.time() - self.start_time
            
        # WALK

        # if elapsed < self.walk_time:
        #     # 1. คำนวณ PID เพื่อหาค่าเลี้ยว (Rotation) มาแก้ทาง
        #     error = WrapRads(self.yaw_setpoint - self.yaw)
        #     rotation = self.controller.Calculate(error)
            
        #     # เดิน
        #     # Calculate motor speeds based on move and turn speeds
        #     self.motor1Speed = (self.moveSpeed + rotation) * self.maxSpeed #Left Start Slower
        #     self.motor2Speed = (self.moveSpeed - rotation) * self.maxSpeed
        #     self.motor3Speed = (self.moveSpeed + rotation) * self.maxSpeed #Left Start Slower
        #     self.motor4Speed = (self.moveSpeed - rotation) * self.maxSpeed
            
        #     self.sendData()
        
        
        
        # else:
        #     self.motor1Speed  = 0
        #     self.motor2Speed  = 0
        #     self.motor3Speed  = 0
        #     self.motor4Speed  = 0
        #     self.sendData()

        #     if not self.stopped:
        #         self.get_logger().info('STOPPED')
        #         self.stopped = True

        if abs(self.total_distance) < self.target_distance:
            error = WrapRads(self.yaw_setpoint - self.yaw)
            rotation = self.controller.Calculate(error)
            
            # self.motor1Speed = (self.moveSpeed + rotation) * self.maxSpeed 
            # self.motor2Speed = (self.moveSpeed - rotation) * self.maxSpeed
            # self.motor3Speed = (self.moveSpeed + rotation) * self.maxSpeed 
            # self.motor4Speed = (self.moveSpeed - rotation) * self.maxSpeed
            
            v_left  = self.moveSpeed + rotation
            v_right = self.moveSpeed - rotation

            rpm_left  = (v_left  / (2 * math.pi * self.wheel_radius)) * 60.0
            rpm_right = (v_right / (2 * math.pi * self.wheel_radius)) * 60.0

            self.motor1Speed = rpm_left
            self.motor3Speed = rpm_left
            self.motor2Speed = rpm_right
            self.motor4Speed = rpm_right
                    
            self.sendData()
            
        else:
            self.motor1Speed  = 0
            self.motor2Speed  = 0
            self.motor3Speed  = 0
            self.motor4Speed  = 0
            self.sendData()
            
            if not self.stopped:
                print(f"\nSTOPPED! Final Distance: {self.total_distance:.4f} meters")
                self.stopped = True
        
    def sendData(self):
        motorspeed_msg = Twist()
       
        motorspeed_msg.linear.x = float(self.motor1Speed) #left front
        motorspeed_msg.linear.y = float(self.motor2Speed) #right front
        motorspeed_msg.angular.x = float(self.motor3Speed) #left rear
        motorspeed_msg.angular.y = float(self.motor4Speed) #right rear
        
        self.send_robot_speed.publish(motorspeed_msg)
        

def main():
    rclpy.init()
    node = AutoWalk()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
