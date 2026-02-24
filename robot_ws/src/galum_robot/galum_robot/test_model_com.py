#!/usr/bin/env python3
import cv2
import numpy as np
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32, Bool
from ultralytics import YOLO

class ClosestEdgeVision(Node):
    def __init__(self):
        super().__init__('vision_node')

        # --- Settings ---
        self.target_offset = 180  # ระยะห่างที่ต้องการขนานกับแปลง (Pixel)

        # --- Publishers ---
        self.error_pub = self.create_publisher(Float32, '/galum/vision_error', 10)
        self.found_pub = self.create_publisher(Bool, '/galum/is_found', 10)

        # --- Subscriber ---
        self.subscription = self.create_subscription(
            CompressedImage,
            '/camera/stream/plot',
            self.process_frame,
            10
        )
        
        self.get_logger().info("Loading YOLO...")
        try:
            self.model = YOLO("rack.pt")
            self.get_logger().info("Model Loaded! Mode: Hug Closest Edge")
        except Exception as e:
            self.get_logger().error(f"Failed to load model: {e}")

    def process_frame(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

        if frame is None: return

        height, width = frame.shape[:2]
        cam_cx = width // 2
        
        # รัน YOLO
        results = self.model.track(frame, persist=True, device="cpu", conf=0.5, verbose=False)
        annotated_frame = results[0].plot()
        
        # วาดเส้นกลางหุ่น (สีเขียว)
        cv2.line(annotated_frame, (cam_cx, 0), (cam_cx, height), (0, 255, 0), 2)
        
        is_found = Bool()
        is_found.data = False
        
        if results[0].boxes is not None and len(results[0].boxes) > 0:
            is_found.data = True
            
            # 1. หาแปลงที่เด่นที่สุด (ใกล้กลางจอที่สุด)
            closest_box = None
            min_dist_center = 9999
            
            for box in results[0].boxes:
                bx1, by1, bx2, by2 = box.xyxy[0].tolist()
                box_cx = (bx1 + bx2) / 2
                dist = abs(box_cx - cam_cx)
                if dist < min_dist_center:
                    min_dist_center = dist
                    closest_box = box

            if closest_box is not None:
                x1, y1, x2, y2 = closest_box.xyxy[0].tolist()
                
                # 2. เช็คว่า "ขอบซ้าย (x1)" หรือ "ขอบขวา (x2)" อันไหนใกล้จมูกหุ่นมากกว่ากัน?
                dist_to_left_edge = abs(cam_cx - x1)
                dist_to_right_edge = abs(cam_cx - x2)
                
                target_x = 0
                debug_color = (0,0,0)
                status_text = ""

                # 3. เลือกเกาะขอบที่ใกล้ที่สุด
                if dist_to_left_edge < dist_to_right_edge:
                    # แปลงอยู่ขวามือ -> เราต้องเกาะขอบซ้ายของแปลง (x1)
                    # เป้าหมาย = ขอบแปลง(x1) - ระยะห่าง(offset)
                    target_x = x1 - self.target_offset
                    
                    status_text = "<< Tracking Left Edge (Rack is Right)"
                    debug_color = (0, 0, 255) # แดง
                    cv2.line(annotated_frame, (int(x1), 0), (int(x1), height), debug_color, 3)

                else:
                    # แปลงอยู่ซ้ายมือ -> เราต้องเกาะขอบขวาของแปลง (x2)
                    # เป้าหมาย = ขอบแปลง(x2) + ระยะห่าง(offset)
                    target_x = x2 + self.target_offset
                    
                    status_text = "Tracking Right Edge (Rack is Left) >>"
                    debug_color = (255, 0, 0) # น้ำเงิน
                    cv2.line(annotated_frame, (int(x2), 0), (int(x2), height), debug_color, 3)
                
                # 4. คำนวณ Error
                # สูตร: Error = ตำแหน่งเรา(cam_cx) - ตำแหน่งที่ควรอยู่(target_x)
                # ถ้าเป็น + แสดงว่าเราอยู่ขวาไป (ต้องเลี้ยวซ้าย)
                # ถ้าเป็น - แสดงว่าเราอยู่ซ้ายไป (ต้องเลี้ยวขวา)
                error_val = float(cam_cx - target_x)
                
                # ส่งค่า
                msg_err = Float32()
                msg_err.data = error_val
                self.error_pub.publish(msg_err)

                # --- Debug Visual ---
                # วาดจุดเป้าหมาย (สีเหลือง)
                cv2.circle(annotated_frame, (int(target_x), height//2), 10, (0, 255, 255), -1)
                # วาดเส้นเชื่อมให้เห็นระยะห่าง
                cv2.line(annotated_frame, (cam_cx, height//2), (int(target_x), height//2), (0, 255, 255), 2)
                
                cv2.putText(annotated_frame, status_text, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
                cv2.putText(annotated_frame, f"Err: {int(error_val)}", (20, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

        self.found_pub.publish(is_found)
        
        cv2.imshow("Closest Edge Tracking", annotated_frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ClosestEdgeVision()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()