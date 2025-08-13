#!/usr/bin/env python3

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
        self.button_circle : float = 0.0        # 1: B
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
        
        self.auto_aim_bool: bool =False
        self.dribble: bool = False 
        self.toggle_shoot_bool : bool = False 
        self.toggle_pass_bool : bool = False
        self.toggle_pass_motor_bool : bool = False
        self.toggle_shoot_2_bool : bool = False
        self.toggle_encoder_bool : bool = False

        self.previous_triangle_state = False
        self.previous_circle_state = False
        self.previous_cross_state = False
        self.previous_square_state = False
        self.previous_l1_state = False
        self.previous_r1_state = False
        self.previous_PressedRightAnalog_state = False



        self.last_macro_button = None  # Stores 'shoot', 'pass', 'dribble', 'auto_aim', 'pass_motor', 'shoot2'


    def update_toggle_encoder(self):
        if self.PressedRightAnalog and not self.previous_PressedRightAnalog_state:
            self.toggle_encoder_bool = not self.toggle_encoder_bool

        self.previous_PressedRightAnalog_state = self.PressedRightAnalog


        


class Joystick(Node):
    def __init__(self):
        super().__init__("joystick")

        self.pub_move = self.create_publisher(
            Twist, "/galum/cmd_move", qos_profile=qos.qos_profile_system_default
        )
        
        self.pub_macro = self.create_publisher(
            Twist, "/galum/cmd_macro", qos_profile=qos.qos_profile_system_default
        )

        self.pub_shoot = self.create_publisher(
            Twist, "/galum/cmd_shoot", qos_profile=qos.qos_profile_system_default
        )

        self.pub_servo = self.create_publisher(
            Twist, "/galum/cmd_servo", qos_profile=qos.qos_profile_system_default
        )

        self.pub_encoder = self.create_publisher(
            Twist, "/galum/cmd_encoder", qos_profile=qos.qos_profile_system_default
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
        
        
        #Macro-----------------------------------------------------------
        
        
        self.gamepad.update_toggle_encoder()
    
        
        # if self.gamepad.button_logo:
        #     self.gamepad.reset_toggles()


        
        

    def sendData(self):
        
        cmd_vel_move = Twist()
        cmd_servo = Twist()
        cmd_encoder = Twist()


        cmd_vel_move.linear.x = float(self.gamepad.lx * self.maxspeed)
        cmd_vel_move.linear.y = float(self.gamepad.ly * self.maxspeed)
        cmd_vel_move.angular.z = float(self.gamepad.rx * self.maxspeed)
        
        
        if self.gamepad.button_share:
            cmd_servo.linear.x = float(1.0)  #Closed Servo
        
        if self.gamepad.button_option:
            cmd_servo.linear.x = float(2.0)  #Opened Servo


        if self.gamepad.toggle_encoder_bool:
            cmd_encoder.linear.x = 2.0 #RPM
        else:
            cmd_encoder.linear.x = 1.0 #Bit
        
        self.pub_encoder.publish(cmd_encoder)
        self.pub_servo.publish(cmd_servo)
        self.pub_move.publish(cmd_vel_move)
       



def main():
    rclpy.init()

    sub = Joystick()
    rclpy.spin(sub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()