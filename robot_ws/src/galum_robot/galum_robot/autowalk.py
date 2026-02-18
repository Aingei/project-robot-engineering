#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
from galum_robot.utilize import *
from galum_robot.controller import *
from motors_interfaces.msg import Motor

class AutoWalk(Node):
    def __init__(self):
        super().__init__('autowalk')

        self.send_robot_speed = self.create_publisher(
            Motor, "/galum/cmd_move/rpm", 10 )
        
        self.send_cmd_vel = self.create_publisher(
            Twist, 
            "/galum/cmd_vel_monitor",  # ตั้งชื่อ topic ใหม่สำหรับดู
            10
        )
        
        self.create_subscription(Twist, "/galum/imu_angle", self.get_robot_angle, 10)
        
        self.timer = self.create_timer(0.05, self.loop)

        # ===== ปรับตรงนี้ =====
        self.moveSpeed = 10.0        # ความเร็ว
        self.walk_time = 10.0      # เดินกี่วินาที
        # =====================
        
        self.turnSpeed = 0.0

        self.previous_time = time.time()
        self.stopped = False
        
        self.yaw = 0.0
        self.yaw_setpoint = 0.0 #self.yaw_setpoint = self.yaw
        self.maxSpeed : float = 1023.0
        
        self.motor1Speed : float = 0
        self.motor2Speed : float = 0
        self.motor3Speed : float = 0
        self.motor4Speed : float = 0
        
        
        self.controller = Controller(kp = 1.0, ki = 0.05, kd = 0.001, errorTolerance= To_Radians(0.5), i_min= -1, i_max= 1)
        

        self.get_logger().info('AUTO WALK START')
    
    def get_robot_angle(self,msg):
        self.yaw = WrapRads(To_Radians(msg.linear.x))

    def get_pid(self,msg):
        self.controller.ConfigPIDF(kp = msg.data[0], ki= msg.data[1], kd=msg.data[2], kf=msg.data[3]) 

    def loop(self):
        
        msg = Twist()
        elapsed = time.time() - self.previous_time
        # CurrentTime = time.time()
        
        # PID
        # yaw_error = WrapRads(self.yaw_setpoint - self.yaw)
        # rotation = self.controller.Calculate(WrapRads(self.yaw_setpoint - self.yaw)) 
        
        # if self.turnSpeed != 0 or (CurrentTime - self.previous_time < 0.45):
        #     rotation = self.turnSpeed
        #     self.yaw_setpoint = self.yaw
        # else:
           
        #     rotation = self.controller.Calculate(WrapRads(self.yaw_setpoint - self.yaw))
        
        # if self.moveSpeed == 0 and self.turnSpeed == 0 :
        #     rotation = 0
        #     self.yaw_setpoint = self.yaw
        
        # self.previous_time = CurrentTime if self.turnSpeed != 0 else self.previous_time

        # self.moveSpeed = msg.linear.x
        # self.turnSpeed = msg.angular.x 

        # self.turnSpeed = self.turnSpeed / 3

            
        # WALK

        if elapsed < self.walk_time:
            # 1. คำนวณ PID เพื่อหาค่าเลี้ยว (Rotation) มาแก้ทาง
            error = WrapRads(self.yaw_setpoint - self.yaw)
            rotation = self.controller.Calculate(error)
            
            # เดิน
            # Calculate motor speeds based on move and turn speeds
            self.motor1Speed = (self.moveSpeed + rotation) * self.maxSpeed #Left Start Slower
            self.motor2Speed = (self.moveSpeed - rotation) * self.maxSpeed
            self.motor3Speed = (self.moveSpeed + rotation) * self.maxSpeed #Left Start Slower
            self.motor4Speed = (self.moveSpeed - rotation) * self.maxSpeed
            
            self.sendData()
        else:
            # หยุด
            # msg.linear.x  = 0.0
            # msg.linear.y  = 0.0
            # msg.angular.x = 0.0
            # msg.angular.y = 0.0
            self.motor1Speed  = 0
            self.motor2Speed  = 0
            self.motor3Speed  = 0
            self.motor4Speed  = 0
            self.sendData()

            if not self.stopped:
                self.get_logger().info('STOPPED')
                self.stopped = True

        
    def sendData(self):
        motorspeed_msg = Motor()
       
        motorspeed_msg.motor1 = float(self.motor1Speed) #left front
        motorspeed_msg.motor2 = float(self.motor2Speed) #right front
        motorspeed_msg.motor3 = float(self.motor3Speed) #left rear
        motorspeed_msg.motor4 = float(self.motor4Speed) #right rear
        
        self.send_robot_speed.publish(motorspeed_msg)
        
        # ==============================
        # กล่องที่ 2: ส่งค่าเดินหน้า (Twist)
        # ==============================
        twist_msg = Twist()
        twist_msg.linear.x = float(self.moveSpeed) # บอกว่าเดินหน้าเท่าไหร่
        twist_msg.angular.z = float(self.turnSpeed) # บอกว่าเลี้ยวเท่าไหร่ (ถ้ามี)

        self.send_cmd_vel.publish(twist_msg)

def main():
    rclpy.init()
    node = AutoWalk()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
