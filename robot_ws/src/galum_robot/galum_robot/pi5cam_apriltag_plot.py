import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class PiCameraSender(Node):
    def __init__(self):
        super().__init__('pi_camera_sender')
       
        # Publisher
        self.publisher_april = self.create_publisher(CompressedImage, '/camera/stream', 10)
        self.publisher_plot = self.create_publisher(CompressedImage, '/camera/stream/plot', 10)
        
        # --- Auto Find Camera Index ---
        self.get_logger().info("Scanning for cameras...")
        valid_cameras = self.find_available_cameras()
        
        if len(valid_cameras) < 2:
            self.get_logger().error(f"Need 2 cameras, but found only {len(valid_cameras)}: {valid_cameras}")
            # ถ้าเจอกล้องเดียว หรือไม่เจอเลย ให้ใช้ค่า Default 0, 1 ไปก่อนเพื่อกัน Code พัง
            idx1 = valid_cameras[0] if len(valid_cameras) > 0 else 0
            idx2 = 1 # สมมติ
        else:
            idx1 = valid_cameras[0]
            idx2 = valid_cameras[1]
            self.get_logger().info(f"Auto-assigned cameras: Camera 1 (April) on index {idx1}, Camera 2 (YOLO) on index {idx2}")

        # เปิดกล้องตาม Index ที่หาเจอ
        self.cap1 = cv2.VideoCapture(idx1)
        self.cap2 = cv2.VideoCapture(idx2)
        
        # Setup Camera 1
        self.setup_camera(self.cap1)
        # Setup Camera 2
        self.setup_camera(self.cap2)
        
        self.timer = self.create_timer(0.05, self.timer_callback) # 20 FPS
        self.get_logger().info(f"Dual Camera Node Started")
        
    def find_available_cameras(self, limit=10):
        """
        วนลูปเช็ค index 0 ถึง 10 ว่าอันไหนเปิดได้และอ่านภาพได้จริง
        """
        available_indices = []
        for i in range(limit):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                # ลองอ่านเฟรม 1 ครั้งเพื่อความชัวร์ (บางทีเปิดได้แต่เป็น Dummy device)
                ret, frame = cap.read()
                if ret:
                    available_indices.append(i)
                    self.get_logger().info(f"Found valid camera at index: {i}")
                cap.release()
            
            # ถ้าเจอครบ 2 ตัวแล้ว หยุดหาได้เลย (เพื่อความเร็ว)
            if len(available_indices) >= 2:
                break
                
        return available_indices

    def setup_camera(self, cap):
        if cap.isOpened():
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
            # ตั้ง Buffer size เป็น 1 ช่วยลด Latency
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    def timer_callback(self):
        # อ่านและส่งกล้อง 1 (AprilTag)
        if self.cap1.isOpened():
            ret1, frame1 = self.cap1.read()
            if ret1:
                self.send_frame(self.publisher_april, frame1)
            
        # อ่านและส่งกล้อง 2 (YOLO Plot)
        if self.cap2.isOpened():
            ret2, frame2 = self.cap2.read()
            if ret2:
                self.send_frame(self.publisher_plot, frame2)
    
    def send_frame(self, publisher, frame):
        # ฟังก์ชันกลางสำหรับบีบอัดและส่งภาพ
        _, buffer = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 50])
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