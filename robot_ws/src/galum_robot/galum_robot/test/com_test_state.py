#!/usr/bin/env python3
# Save as: pc_vision_sender.py (Run on PC)
import cv2
import numpy as np
import pupil_apriltags
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO

class PCVisionSender(Node):
    def __init__(self):
        super().__init__('pc_vision_sender')

        # --- Settings ---
        self.yolo_model = "rack.pt"  # ชื่อ Model แปลงของคุณ
        
        # --- Pub/Sub ---
        # รับภาพจาก Pi
        self.create_subscription(CompressedImage, '/camera/stream', self.process_frame, 10)
        
        # ส่งข้อมูลที่ประมวลผลแล้วกลับไป Pi (รวมทุกอย่างใน array เดียว)
        self.data_pub = self.create_publisher(Float32MultiArray, '/galum/vision_packet', 10)

        # --- AI Setup ---
        self.at_detector = pupil_apriltags.Detector(families='tagStandard52h13')
        self.model = YOLO(self.yolo_model)
        
        self.get_logger().info("PC Vision Started: AprilTag + YOLO Mode")

    def decode_tag(self, tag_id):
        # ถอดรหัส 40115 -> 40, 1(5cm), 15
        s = str(int(tag_id)).zfill(5)
        if len(s) != 5: return 0, 0, 0
        
        first_dist = int(s[0:2])  # 40
        gap_code = s[2]           # 1
        interval = int(s[3:5])    # 15
        
        gap_cm = {'1':5, '2':10, '3':15, '4':20, '5':25}.get(gap_code, 0)
        
        return first_dist, gap_cm, interval

    def process_frame(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None: return

        height, width, _ = frame.shape
        center_x = width // 2

        # ===========================
        # 1. Process AprilTag
        # ===========================
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.at_detector.detect(gray)
        
        tag_found = 0.0
        tag_error = 0.0
        tag_size = 0.0
        val_first = 0.0
        val_gap = 0.0
        val_interval = 0.0

        if detections:
            tag = max(detections, key=lambda x: x.decision_margin) # เอาอันชัดสุด
            tag_found = 1.0
            
            # คำนวณ Error
            tag_cx = int(tag.center[0])
            tag_error = float(center_x - tag_cx)
            
            # คำนวณ Size (ความกว้าง)
            pts = tag.corners
            width_px = abs(pts[1][0] - pts[0][0])
            tag_size = float(width_px)
            
            # ถอดรหัส
            val_first, val_gap, val_interval = self.decode_tag(tag.tag_id)

            # Debug Draw
            cv2.polylines(frame, [pts.astype(int)], True, (0, 255, 0), 2)
            cv2.putText(frame, f"CODE: {tag.tag_id}", (tag_cx, int(tag.center[1])), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        # ===========================
        # 2. Process YOLO (Row)
        # ===========================
        # ใช้ Track เพื่อความนิ่ง
        results = self.model.track(frame, persist=True, verbose=False, device='cpu') 
        
        row_error = 0.0
        # row_found เช็คจาก detections
        
        if results[0].boxes:
            # หาแปลงที่ใกล้กลางจอที่สุด (เหมือนโค้ดเดิมของคุณ)
            boxes = results[0].boxes
            min_dist = 9999
            target_x = center_x
            
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                box_cx = (x1 + x2) / 2
                dist = abs(center_x - box_cx)
                
                if dist < min_dist:
                    min_dist = dist
                    # Logic เดิม: เลือกขอบที่ใกล้
                    d_left = abs(center_x - x1)
                    d_right = abs(center_x - x2)
                    offset = 180 # ระยะห่างจากแปลง
                    
                    if d_left < d_right: 
                        target_x = x1 - offset # เกาะขอบซ้าย
                    else:
                        target_x = x2 + offset # เกาะขอบขวา
            
            row_error = float(center_x - target_x)
            
            # Debug Draw YOLO
            annotated_frame = results[0].plot()
            cv2.imshow("PC Vision", annotated_frame) # โชว์รูปที่มี YOLO
        else:
            cv2.imshow("PC Vision", frame) # โชว์รูป Tag

        cv2.waitKey(1)

        # ===========================
        # 3. Send Packet to Pi
        # ===========================
        # Packet Structure: [TagFound, TagErr, TagSize, RowErr, FirstDist, Gap, Interval]
        msg_out = Float32MultiArray()
        msg_out.data = [
            tag_found,      # 0
            tag_error,      # 1
            tag_size,       # 2
            row_error,      # 3
            float(val_first), # 4 (40cm)
            float(val_gap),   # 5 (5cm)
            float(val_interval) # 6 (15cm)
        ]
        self.data_pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = PCVisionSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()