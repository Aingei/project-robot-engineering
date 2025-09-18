#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int16MultiArray , Float32MultiArray
from geometry_msgs.msg import Twist 
from sensor_msgs.msg import Joy
from rclpy import qos

class Gamepad:
    def __init__(self):
        #Axes:--------------------------------------------------------
        
        self.lx : float = 0.0                   # 0: Left X-Axis
        self.ly : float = 0.0                   # 1: Left Y-Axis
        self.l2 : float = 0.0                   # 2: LT
        self.rx : float = 0.0                   # 3: Right X-Axis
        self.ry : float = 0.0                   # 4: Right Y-Axis
        self.r2 : float = 0.0                   # 5: RT
        self.dpadLeftRight : float = 0.0        # 6: Dpad Left and Right
        self.dpadUpDown : float = 0.0           # 7: Dpad Up and Down
        
        #Buttons:-------------------------------------------------------
        
        self.button_cross : float = 0.0         # 0: A
        self.button_circle : float = 0.0         # 1: B
        self.button_triangle : float = 0.0      # 2: X
        self.button_square : float = 0.0        # 3: Y
        self.l1 : float = 0.0                   # 4: LB
        self.r1 : float = 0.0                   # 5: RB
        self.button_share : float = 0.0         # 8: -
        self.button_option : float = 0.0        # 9: +
        self.button_logo : float = 0.0          # 10: Logo
        self.PressedLeftAnalog : float = 0.0    # 11: Pressed Left Analog
        self.PressedRightAnalog : float = 0.0   # 12: Pressed Right Analog

        #----------------------------------------------------------------


class Joystick(Node):
    def __init__(self):
        super().__init__("joystick")

        self.pub_move = self.create_publisher(
            Twist, "/galum/cmd_move", qos_profile=qos.qos_profile_system_default
        )
        
        self.pub_stepper = self.create_publisher(
            Twist, "/galum/stepper", qos_profile=qos.qos_profile_system_default
        )

         self.pub_servo = self.create_publisher(
            Twist, "/galum/servo", qos_profile=qos.qos_profile_system_default
        )
        
        self.create_subscription(
            Joy, '/galum/joy', self.joy, qos_profile=qos.qos_profile_sensor_data # 10
        )

        self.gamepad = Gamepad()
        self.maxspeed : float = 1.0
        
        self.sent_data_timer = self.create_timer(0.01, self.sendData)

    def joy(self, msg):
        
        #Axes:--------------------------------------------------------
        
        self.gamepad.lx = float(msg.axes[0] * -1)                   # 0: Left X-Axis
        self.gamepad.ly = float(msg.axes[1])                        # 1: Left Y-Axis
        self.gamepad.l2 = float((msg.axes[2] + 1)/ 2)               # 2: L2
        self.gamepad.rx = float(msg.axes[3] * -1)                   # 3: Right X-Axis
        self.gamepad.ry = float(msg.axes[4])                        # 4: Right Y-Axis
        self.gamepad.r2 = float((msg.axes[5] + 1)/ 2)               # 5: R2
        self.gamepad.dpadLeftRight  = float(msg.axes[6])            # 6: Dpad Left and Right
        self.gamepad.dpadUpDown     = float(msg.axes[7])            # 7: Dpad Up and Down
        
        #Buttons:-------------------------------------------------------

        self.gamepad.button_cross    = float(msg.buttons[0])        # 0: 
        self.gamepad.button_circle   = float(msg.buttons[1])        # 1:
        self.gamepad.button_triangle = float(msg.buttons[2])        # 2:
        self.gamepad.button_square   = float(msg.buttons[3])        # 3:
        self.gamepad.l1              = float(msg.buttons[4])        # 4:
        self.gamepad.r1              = float(msg.buttons[5])        # 5:
        self.gamepad.button_share    = float(msg.buttons[6])        # 8:
        self.gamepad.button_option   = float(msg.buttons[7])        # 9:
        self.gamepad.button_logo     = float(msg.buttons[8])       # 10:
        self.gamepad.PressedLeftAnalog  = float(msg.buttons[9])    # 11:
        self.gamepad.PressedRightAnalog = float(msg.buttons[10])    # 12:
        
     
        if self.gamepad.button_logo:
            self.gamepad.reset_toggles()

    
    def sendData(self):
        
        cmd_vel_move = Twist()
        cmd_servo = Twist()
        cmd_stepper = Twist()

        cmd_vel_move.linear.x = float(self.gamepad.ly * self.maxspeed)
        cmd_vel_move.angular.z = float(self.gamepad.rx * self.maxspeed)
        
        if self.gamepad.button_cross:
            cmd_servo.linear.x = float(1.0)  #Closed Servo
        
        if self.gamepad.button_circle:
            cmd_servo.linear.x = float(2.0)  #Opened Servo
            
        speed_r = self.gamepad.r2 * self.maxspeed_stepper
        speed_l = self.gamepad.l2 * self.maxspeed_stepper

        if self.gamepad.r1 and not self.gamepad.l1:
            # ตามเข็ม: ให้เป็นค่าบวก
            cmd_stepper.linear.x = + (speed_r if speed_r > 0.0 else self.maxspeed_stepper * 0.5)
        elif self.gamepad.l1 and not self.gamepad.r1:
            # ทวนเข็ม: ให้เป็นค่าลบ
            cmd_stepper.linear.x = - (speed_l if speed_l > 0.0 else self.maxspeed_stepper * 0.5)
        else:
            # ไม่กด/กดพร้อมกัน -> หยุด
            cmd_stepper.linear.x = 0.0
        
        
        self.pub_move.publish(cmd_vel_move)
        self.pub_servo.publish(cmd_servo)
        self.pub_stepper.publish(cmd_stepper)

def main():
    rclpy.init()

    sub = Joystick()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()