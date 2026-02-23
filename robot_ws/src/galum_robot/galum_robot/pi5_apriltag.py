import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class PiCameraSender(Node):
    def __init__(self):
        super().__init__('pi_camera_sender')
        self.publisher = self.create_publisher(CompressedImage, '/camera/stream', 10)
        self.cap = cv2.VideoCapture(0)
        
        # ปรับความละเอียดที่ 640x480 (ชัดพอสำหรับ AprilTag และไม่หนักเครื่อง)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        
        self.timer = self.create_timer(0.05, self.publish_frame) # 20 FPS

    def publish_frame(self):
        ret, frame = self.cap.read()
        if ret:
            # บีบอัดเป็น JPEG เพื่อให้ส่งผ่าน Network ได้ลื่น
            _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = buffer.tobytes()
            self.publisher.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(PiCameraSender())
    rclpy.spin(node)
    rclpy.shutdown()