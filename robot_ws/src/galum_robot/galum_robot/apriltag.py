import cv2
import pupil_apriltags
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class AprilTagCompressedPublisher(Node):

    def __init__(self):
        super().__init__('apriltag_camera_node')

        self.publisher = self.create_publisher(
            CompressedImage,
            '/camera/image/compressed',
            10 
        )

        self.detector = pupil_apriltags.Detector(families='tagStandard52h13')
        self.cap = cv2.VideoCapture(1)

        self.timer = self.create_timer(0.05, self.process_frame)

        self.get_logger().info("Compressed camera publisher started")

    # ==============================
    # 🔹 เพิ่มฟังก์ชัน decode ตรงนี้
    # ==============================
    def decode_cabbage_data(self, tag_id):

        s = str(tag_id).zfill(5)
        if len(s) != 5:
            return None

        data = {
            "id": tag_id,
            "planting_dist": int(s[0:2]),
            "gap": {'1':5, '2':10, '3':15, '4':20, '5':25}.get(s[2], 0),
            "interval": int(s[3:5])
        }
        return data

    # ==============================
    # 🔹 main processing
    # ==============================
    def process_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning("Cannot read camera")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)

        for tag in detections:
            pts = tag.corners.reshape((-1, 1, 2)).astype(int)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

            cv2.putText(frame, f"ID: {tag.tag_id}",
                        (pts[0][0][0], pts[0][0][1]-10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (0, 255, 0), 2)

            # 🔥 เพิ่มส่วน decode ตรงนี้
            result = self.decode_cabbage_data(tag.tag_id)

            if result:
                self.get_logger().info(
                    f"move: distance {result['planting_dist']} | "
                    f"เว้น {result['gap']} | {result['interval']}"
                )

        # 🔥 encode JPEG
        success, buffer = cv2.imencode('.jpg', frame)
        if not success:
            return

        msg = CompressedImage()
        msg.format = "jpeg"
        msg.data = buffer.tobytes()

        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AprilTagCompressedPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
