import cv2
import numpy as np
import pupil_apriltags
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

class PCProcessor(Node):
    def __init__(self):
        super().__init__('pc_processor_node')

        # รับภาพจาก Pi 5
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/stream',
            self.process_frame,
            qos_profile=qos.qos_profile_sensor_data
        )

        self.detector = pupil_apriltags.Detector(families='tagStandard52h13')
        self.get_logger().info("PC Processor started - Waiting for stream...")

    def decode_cabbage_data(self, tag_id):
        s = str(tag_id).zfill(5)
        if len(s) != 5: return None
        data = {
            "id": tag_id,
            "planting_dist": int(s[0:2]),
            "gap": {'1':5, '2':10, '3':15, '4':20, '5':25}.get(s[2], 0),
            "interval": int(s[3:5])
        }
        return data

    def process_frame(self, msg):
        # 🔥 แปลงจาก CompressedImage เป็น OpenCV Frame
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
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

            result = self.decode_cabbage_data(tag.tag_id)

            if result:
                # โชว์ค่าใน Terminal ของเครื่องคอม
                self.get_logger().info(
                    f"move: distance {result['planting_dist']} | "
                    f"เว้น {result['gap']} | {result['interval']}"
                )
                # วาดค่าลงบนภาพด้วย
                cv2.putText(frame, f"Dist: {result['planting_dist']} Gap: {result['gap']}",
                            (pts[0][0][0], pts[0][0][1]+20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # 🔥 แสดงภาพบนหน้าจอคอม
        cv2.imshow("Stream from Pi 5 (Processed)", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PCProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()