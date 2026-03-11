#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TestStepper(Node):
    def __init__(self):
        super().__init__('test_stepper_node')
        self.pub = self.create_publisher(Twist, '/galum/stepper/angle', 10)
        
        self.is_cw = True
        self.pulses = 800.0  # จำนวนพัลส์ที่ต้องการเทส (เปลี่ยนค่าได้)
        
        # ส่งคำสั่งทุกๆ 3 วินาที (ต้องเผื่อเวลาให้มอเตอร์หมุนเสร็จด้วย)
        self.create_timer(3.0, self.timer_callback)
        self.get_logger().info("กำลังเริ่มเทส Stepper... (กด Ctrl+C เพื่อหยุด)")

    def timer_callback(self):
        msg = Twist()
        
        if self.is_cw:
            self.get_logger().info(f"สั่ง Stepper ลง หมุน ตามเข็ม (CW) {self.pulses} พัลส์")
            msg.linear.x = -1.0   # 1.0 = ตามเข็ม (CW)
        else:
            self.get_logger().info(f"สั่ง Stepper ขึ้น หมุน ทวนเข็ม (CCW) {self.pulses} พัลส์")
            msg.linear.x = 1.0  # -1.0 = ทวนเข็ม (CCW)
            
        msg.linear.y = self.pulses
        self.pub.publish(msg)
        
        # สลับทิศทางในรอบถัดไป
        self.is_cw = not self.is_cw

def main(args=None):
    rclpy.init(args=args)
    node = TestStepper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("หยุดการเทส Stepper")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()