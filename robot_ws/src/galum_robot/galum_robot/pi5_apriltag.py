import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class PiCameraSender(Node):
    def __init__(self, camera_id=0, topic_name='camera'):
        super().__init__('pi_camera_sender')
       
        # Publisher 1: สำหรับ AprilTag (cap1)
        self.publisher_april = self.create_publisher(CompressedImage, '/camera/stream', 10)
        # Publisher 2: สำหรับ YOLO/Plot (cap2)
        self.publisher_plot = self.create_publisher(CompressedImage, '/camera/stream/plot', 10)
        
        # เปิดกล้อง 2 ตัว
        self.cap1 = cv2.VideoCapture(0) # กล้องหลัก
        self.cap2 = cv2.VideoCapture(1) # กล้องรอง
        
        # ตั้งค่าความละเอียด 640x480 ทั้งคู่
        self.cap1.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap1.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.cap2.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap2.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.timer = self.create_timer(0.05, self.timer_callback) # 20 FPS
        self.get_logger().info(f"Dual Camera Node Started")
        
    def timer_callback(self):
        # อ่านและส่งกล้อง 1 (AprilTag)
        ret1, frame1 = self.cap1.read()
        if ret1:
            self.send_frame(self.publisher_april, frame1)
            
        # อ่านและส่งกล้อง 2 (YOLO Plot)
        ret2, frame2 = self.cap2.read()
        if ret2:
            self.send_frame(self.publisher_plot, frame2)
    
    def send_frame(self, publisher, frame):
        # ฟังก์ชันกลางสำหรับบีบอัดและส่งภาพ
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50]) # ลด Quality เหลือ 50 เพื่อความลื่นไหล
        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = buffer.tobytes()
        publisher.publish(msg)

def main():
    rclpy.init()
    node = PiCameraSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap1.release()
        node.cap2.release()
        rclpy.shutdown()

if __name__ == '__main__':
    main()