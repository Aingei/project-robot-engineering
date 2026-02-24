import cv2
import numpy as np
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

class YoloProcessor(Node):
    def __init__(self):
        super().__init__('yolo_processor_node')

        # รับภาพจาก Topic ของกล้อง 2
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/stream/plot',
            self.process_frame,
            10
        )
        
        self.get_logger().info("Loading YOLO model...")
        try:
            self.model = YOLO("rack.pt")
            self.get_logger().info("YOLO Model loaded successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")

        self.get_logger().info("YOLO Processor started - Waiting for plot stream...")

    def process_frame(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None:
            return

        height, width = frame.shape[:2]
        cam_cx = width // 2
        
        # รัน YOLO Track
        results = self.model.track(frame, persist=True, device="cpu", conf=0.6, verbose=False)
        annotated_frame = results[0].plot()
        
        # วาดเส้น Center
        cv2.line(annotated_frame, (cam_cx, 0), (cam_cx, height), (0, 255, 0), 2)
        
        if results[0].boxes is not None:
            for box in results[0].boxes:
                # พิกัด bbox
                x1, y1, x2, y2 = box.xyxy[0].tolist()

                # center บน และ ล่าง ของ bbox
                top_center = (int((x1 + x2) / 2), int(y1))
                bottom_center = (int((x1 + x2) / 2), int(y2))

                # วาดจุด
                cv2.circle(annotated_frame, top_center, 6, (0, 0, 255), -1)   # บน = แดง
                cv2.circle(annotated_frame, bottom_center, 6, (255, 0, 0), -1) # ล่าง = น้ำเงิน

                # center bbox & Error calculation
                cx, cy, w, h = box.xywh[0].tolist()

                dx = cx - cam_cx
                dy = height - cy
                
                # ป้องกัน dy เป็น 0
                if dy == 0: dy = 0.001
                
                angle_error = math.degrees(math.atan2(dx, dy))
                lateral_error = cx - cam_cx

                # แสดง text
                cv2.putText(annotated_frame, f"Heading Error: {angle_error:.2f} deg", (20, 50),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

                cv2.putText(annotated_frame, f"Lat Error: {int(lateral_error)} px", (20, 80),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                            
                # Log ค่าออกมาดู (ถ้าต้องการ)
                # self.get_logger().info(f"H_Err: {angle_error:.2f} | L_Err: {lateral_error:.2f}")

        cv2.imshow("YOLO Tracking Stream", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = YoloProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()