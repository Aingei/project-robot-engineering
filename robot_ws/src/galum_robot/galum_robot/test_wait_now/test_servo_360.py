#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
import time

def main():
    rclpy.init()
    node = rclpy.create_node('test_servo4')
    pub = node.create_publisher(Twist, '/galum/servo/angle', 10)
    msg = Twist()
    
    msg.linear.x = 90.0
    msg.linear.y = 90.0
    msg.linear.z = 90.0
    
    #servo 2,4
    
    print("=== Testing Servo 4 (angular.x) 360 Degree ===")
    try:
        while rclpy.ok():
            print("Servo 4: เจาะดิน (180.0) เป็นเวลา 3 วินาที")
            msg.angular.x = 180.0
            pub.publish(msg)
            time.sleep(3)

            print("Servo 4: หยุดเจาะ (90.0)")
            msg.angular.x = 90.0
            pub.publish(msg)
            time.sleep(2)

            print("Servo 4: หมุนถอยกลับ (0.0) เป็นเวลา 3 วินาที")
            msg.angular.x = 0.0
            pub.publish(msg)
            time.sleep(3)
            
            print("Servo 4: หยุดเจาะ (90.0)")
            msg.angular.x = 90.0
            pub.publish(msg)
            time.sleep(2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()