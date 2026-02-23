#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
import time
import math
from rclpy import qos
from galum_robot.utilize import *
from galum_robot.controller import *

class AutoWalk(Node):
    def __init__(self):
        super().__init__('autowalk')

        # --- Publishers ---
        self.send_robot_speed = self.create_publisher(
            Twist, "/galum/cmd_move/rpm", qos_profile=qos.qos_profile_system_default 
        )
        self.distance_pub = self.create_publisher(
            Float32, "/galum/current_distance", qos_profile=qos.qos_profile_system_default
        )
        
        # --- Subscribers ---
        self.create_subscription(
            Twist, "/galum/imu_angle", self.get_robot_angle, qos_profile=qos.qos_profile_sensor_data
        )
        self.create_subscription(
            Twist, "/galum/encoder", self.encoder_callback, qos_profile=qos.qos_profile_sensor_data
        )
        
        self.timer = self.create_timer(0.05, self.loop)

        # --- Movement Settings ---
        self.target_distance = 3.0  # Target: 1 Meter
        self.moveSpeed = 0.5        # Forward Speed (m/s)
        
        self.yaw = 0.0
        self.yaw_setpoint = 0.0 
        
        self.motor1Speed = 0.0
        self.motor2Speed = 0.0
        self.motor3Speed = 0.0
        self.motor4Speed = 0.0
        
        self.stopped = False
        
        # --- Encoder Configuration ---
        self.wheel_radius = 0.05        # 50mm
        self.ticks_per_rev = 7436       # Pulse per revolution
        
        circumference = 2 * math.pi * self.wheel_radius
        self.m_per_tick = circumference / self.ticks_per_rev
        
        self.prev_ticks = [0, 0, 0, 0]
        self.total_distance = 0.0
        self.first_run = True   
        
        # PID Controller
        self.controller = Controller(kp = 1.0, ki = 0.05, kd = 0.001, errorTolerance= To_Radians(0.5), i_min= -1, i_max= 1)
        
        self.get_logger().info(f'AUTO WALK START. Target: {self.target_distance}m')
    
    def encoder_callback(self, msg):
        # Current Ticks from Message
        # Index 0: FL, 1: FR, 2: RL, 3: RR
        current_ticks = [
            msg.linear.x, msg.linear.y, msg.angular.x, msg.angular.y
        ]
        
        if self.first_run:
            self.prev_ticks = current_ticks
            self.first_run = False
            return
        
        current_dist_step = 0.0
        
        for i in range(4):
            diff = current_ticks[i] - self.prev_ticks[i]
            
            # === POLARITY FIX ===
            # You stated: FL (idx 0) and RR (idx 3) are negative when moving forward.
            # We invert them so FORWARD is always POSITIVE.
            if i == 0 or i == 3:
                diff = -diff
            # ====================

            # Convert ticks to meters
            dist_m = diff * self.m_per_tick
            current_dist_step += dist_m
            
        # Average distance of 4 wheels
        avg_distance_step = current_dist_step / 4.0
        
        # Update Total Distance
        self.total_distance += avg_distance_step
        self.prev_ticks = current_ticks

        # Publish for debugging
        dist_msg = Float32()
        dist_msg.data = self.total_distance
        self.distance_pub.publish(dist_msg)
            
    def get_robot_angle(self, msg):
        self.yaw = WrapRads(To_Radians(msg.linear.x))

    def loop(self):
        if self.stopped:
            return

        # Check if we reached target distance
        # We use abs() here just in case we overshoot slightly negative
        if abs(self.total_distance) < self.target_distance:
            
            # 1. Calculate Heading Correction (PID)
            error = WrapRads(self.yaw_setpoint - self.yaw)
            rotation = self.controller.Calculate(error)
            
            # 2. Calculate Wheel Speeds
            v_left  = self.moveSpeed + rotation
            v_right = self.moveSpeed - rotation

            # 3. Convert m/s to RPM
            rpm_left  = (v_left  / (2 * math.pi * self.wheel_radius)) * 60.0
            rpm_right = (v_right / (2 * math.pi * self.wheel_radius)) * 60.0

            self.motor1Speed = rpm_left
            self.motor3Speed = rpm_left
            self.motor2Speed = rpm_right
            self.motor4Speed = rpm_right
            
            self.sendData()
            
            # Print Status
            print(f"Dist: {self.total_distance:.3f} / {self.target_distance} m | RPM: {int(rpm_left)}", end='\r')
            
        else:
            # STOP
            self.motor1Speed  = 0.0
            self.motor2Speed  = 0.0
            self.motor3Speed  = 0.0
            self.motor4Speed  = 0.0
            self.sendData()
            
            if not self.stopped:
                print(f"\nSTOPPED! Final Distance: {self.total_distance:.4f} meters")
                self.stopped = True
        
    def sendData(self):
        motorspeed_msg = Twist()
        motorspeed_msg.linear.x = float(self.motor1Speed)
        motorspeed_msg.linear.y = float(self.motor2Speed)
        motorspeed_msg.angular.x = float(self.motor3Speed)
        motorspeed_msg.angular.y = float(self.motor4Speed)
        self.send_robot_speed.publish(motorspeed_msg)

def main():
    rclpy.init()
    node = AutoWalk()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()