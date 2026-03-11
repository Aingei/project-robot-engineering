#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
import time

def main():
    rclpy.init()
    node = rclpy.create_node('test_servo1')
    pub = node.create_publisher(Twist, '/galum/servo/angle', 10)
    msg = Twist()
    #servo 1,3
    # เซ็ตตัวอื่นให้เป็นค่ากลาง (90) เพื่อไม่ให้ขยับมั่ว
    
    msg.linear.x = 90.0
    msg.linear.y = 90.0
    #msg.linear.z = 90.0
    msg.angular.x = 90.0

    print("=== Testing Servo 1 (linear.x) 180 Degree ===")
    try:
        while rclpy.ok():
            print("Servo 1: กางออก 0 องศา")
            msg.linear.z = 0.0
            pub.publish(msg)
            time.sleep(2)

            print("Servo 1: เก็บแขน 100 องศา")
            msg.linear.z = 100.0
            pub.publish(msg)
            time.sleep(2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()