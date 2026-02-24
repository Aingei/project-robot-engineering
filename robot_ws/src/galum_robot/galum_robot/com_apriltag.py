import cv2
import numpy as np
import math
import pupil_apriltags
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from ultralytics import YOLO

class PCProcessor(Node):
    def __init__(self):
        super().__init__('pc_processor_node')

        # --- ส่วนที่ 1: รับภาพ AprilTag (จาก cap1) ---
        self.sub_april = self.create_subscription(
            CompressedImage,
            '/camera/stream',
            self.process_apriltag_frame,
            10
        )

        # --- ส่วนที่ 2: รับภาพ YOLO/Rack (จาก cap2) ---
        self.sub_yolo = self.create_subscription(
            CompressedImage,
            '/camera/stream/plot',
            self.process_rack_frame,
            10
        )

        # Setup AprilTag Detector
        self.detector = pupil_apriltags.Detector(families='tagStandard52h13')
        
        # Setup YOLO Model
        self.get_logger().info("Loading YOLO model...")
        self.model = YOLO("rack.pt") # ตรวจสอบ path ไฟล์ rack.pt ให้ถูกต้อง
        
        self.get_logger().info("PC Processor started - Waiting for BOTH streams...")

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

    # --- Callback สำหรับ AprilTag (กล้อง 1) ---
    def process_apriltag_frame(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None: return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.detector.detect(gray)

        for tag in detections:
            pts = tag.corners.reshape((-1, 1, 2)).astype(int)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            
            result = self.decode_cabbage_data(tag.tag_id)
            if result:
                cv2.putText(frame, f"Dist: {result['planting_dist']} Gap: {result['gap']}",
                            (pts[0][0][0], pts[0][0][1]-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow("Camera 1: AprilTag", frame)
        cv2.waitKey(1)

    # --- Callback สำหรับ YOLO Rack (กล้อง 2) ---
    def process_rack_frame(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None: return

        # YOLO Tracking Logic
        height, width = frame.shape[:2]
        cam_cx = width // 2
        
        # รัน YOLO (Track mode)
        results = self.model.track(frame, persist=True, device="cpu", conf=0.6, verbose=False)
        annotated_frame = results[0].plot()

        # วาดเส้น Center กล้อง
        cv2.line(annotated_frame, (cam_cx, 0), (cam_cx, height), (0, 255, 0), 2)

        if results[0].boxes is not None and len(results[0].boxes) > 0:
            # เลือก Box ที่มั่นใจที่สุด หรือ Box แรก
            box = results[0].boxes[0]
            
            # พิกัด bbox
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            # Center บน และ ล่าง ของ bbox
            top_center = (int((x1 + x2) / 2), int(y1))
            bottom_center = (int((x1 + x2) / 2), int(y2))

            # วาดจุด
            cv2.circle(annotated_frame, top_center, 6, (0, 0, 255), -1)   # บน = แดง
            cv2.circle(annotated_frame, bottom_center, 6, (255, 0, 0), -1) # ล่าง = น้ำเงิน

            # คำนวณ Error
            cx, cy, w, h = box.xywh[0].tolist()
            dx = cx - cam_cx
            dy = height - cy
            
            # ป้องกันการหารด้วย 0
            if dy == 0: dy = 0.001
            
            angle_error = math.degrees(math.atan2(dx, dy))
            lateral_error = cx - cam_cx

            # แสดงค่าบนหน้าจอ
            cv2.putText(annotated_frame, f"Heading: {angle_error:.2f} deg", (20, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            cv2.putText(annotated_frame, f"Lat Error: {int(lateral_error)} px", (20, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
            
            # (Optional) Log ค่าลง Terminal เพื่อ Debug
            # self.get_logger().info(f"YOLO: H_Err={angle_error:.2f}, L_Err={lateral_error}")

        cv2.imshow("Camera 2: Rack Tracking", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PCProcessor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()