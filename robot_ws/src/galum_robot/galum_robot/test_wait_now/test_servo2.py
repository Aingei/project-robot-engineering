#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist
import time

def main():
    rclpy.init()
    node = rclpy.create_node('servo4_spin')
    pub = node.create_publisher(Twist, '/galum/servo/angle', 10)

    msg = Twist()

    # ตั้ง servo อื่นให้นิ่ง
    msg.linear.x = 90.0
    #msg.linear.y = 90.0
    msg.linear.z = 90.0
    msg.angular.x = 90.0

    try:
        while rclpy.ok():

            print("Servo 4: หมุน")
            msg.linear.y = 180.0
            pub.publish(msg)
            time.sleep(5)

            print("Servo 4: หยุด")
            msg.linear.y = 90.0
            pub.publish(msg)
            time.sleep(3)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()