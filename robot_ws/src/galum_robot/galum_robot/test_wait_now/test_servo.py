#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TestServo(Node):
    def __init__(self):
        super().__init__('test_servo_node')
        self.pub = self.create_publisher(Twist, '/galum/servo/angle', 10)
        
        # ลำดับมุมที่จะใช้เทส
        self.test_angles = [0.0, 90.0, 180.0, 90.0]
        self.index = 0
        
        # ตั้งเวลาเทสทุกๆ 2 วินาที
        self.create_timer(2.0, self.timer_callback)
        self.get_logger().info("กำลังเริ่มเทส Servo... (กด Ctrl+C เพื่อหยุด)")

    def timer_callback(self):
        angle = self.test_angles[self.index]
        self.get_logger().info(f"สั่ง Servo ทั้ง 3 ตัวไปที่มุม: {angle} องศา")
        
        msg = Twist()
        msg.linear.x = angle  # Servo 1
        msg.linear.y = angle  # Servo 2
        msg.linear.z = angle  # Servo 3
        self.pub.publish(msg)
        
        # เลื่อน index ไปมุมถัดไป
        self.index = (self.index + 1) % len(self.test_angles)

def main(args=None):
    rclpy.init(args=args)
    node = TestServo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("หยุดการเทส Servo")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()