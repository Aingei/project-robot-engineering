#!/usr/bin/env python3
import cv2
import numpy as np
import math
import pupil_apriltags
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray
from ultralytics import YOLO
# from rclpy.qos import qos_profile_sensor_data
from rclpy import qos

class PCVisionCombined(Node):
    def __init__(self):
        super().__init__('pc_vision_combined')

        # --- Settings ---
        self.yolo_model = "rack.pt" 
        
        # 1. รับภาพ AprilTag
        self.create_subscription(CompressedImage, '/camera/stream', self.process_april, qos_profile=qos.qos_profile_sensor_data)
        
        # 2. รับภาพ YOLO
        self.create_subscription(CompressedImage, '/camera/stream/plot', self.process_yolo, qos_profile=qos.qos_profile_sensor_data)
        
        # Publisher
        self.data_pub = self.create_publisher(Float32MultiArray, '/galum/vision_packet', qos_profile=qos.qos_profile_sensor_data)
        self.create_timer(0.05, self.send_packet)

        # AI
        self.at_detector = pupil_apriltags.Detector(families='tagStandard52h13')
        self.get_logger().info("Loading YOLO to GPU...")
        self.model = YOLO(self.yolo_model)
        
        # Variables
        self.tag_found = 0.0
        self.tag_error = 0.0
        self.tag_size = 0.0
        self.plant_dist = 0.0
        self.plant_gap = 0.0
        self.plant_interval = 0.0
        self.yolo_lat_error = 0.0   

        self.get_logger().info("PC Vision Combined Started (Horizontal Line Mode)")

    def decode_cabbage_data(self, tag_id):
        s = str(tag_id).zfill(5)
        if len(s) != 5: return 0, 0, 0
        p_dist = int(s[0:2])
        gap = {'1':5, '2':10, '3':15, '4':20, '5':25}.get(s[2], 0)
        interval = int(s[3:5])
        return p_dist, gap, interval

    def process_april(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None: return

        center_x = frame.shape[1] // 2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.at_detector.detect(gray)
        
        found = 0.0
        if detections:
            tag = max(detections, key=lambda x: x.decision_margin)
            found = 1.0
            self.tag_error = float(center_x - int(tag.center[0]))
            self.tag_size = float(abs(tag.corners[1][0] - tag.corners[0][0]))
            
            p_dist, gap, interval = self.decode_cabbage_data(tag.tag_id)
            self.plant_dist = float(p_dist)
            self.plant_gap = float(gap)
            self.plant_interval = float(interval)

            cv2.polylines(frame, [tag.corners.astype(int)], True, (0, 255, 0), 2)
            cv2.putText(frame, f"Dist:{p_dist} Gap:{gap}", (int(tag.center[0]), int(tag.center[1])), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        self.tag_found = found
        cv2.imshow("Cam 1: AprilTag", frame)
        cv2.waitKey(1)

    def process_yolo(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None: return

        height, width = frame.shape[:2]
        cam_cx = width // 2
        
        # รัน YOLO (device=0 คือ GPU)
        results = self.model.track(frame, persist=True, device=0, imgsz=320, verbose=False)
        annotated_frame = results[0].plot()
        
        lat_err = 0.0

        if results[0].boxes:
            closest_box = None
            min_dist = 9999
            
            for box in results[0].boxes:
                cx, cy, w, h = box.xywh[0].tolist()
                if abs(cam_cx - cx) < min_dist:
                    min_dist = abs(cam_cx - cx)
                    closest_box = box

            if closest_box:
                cx, cy, w, h = closest_box.xywh[0].tolist()
                
                # คำนวณ Error (ระยะห่างซ้ายขวาเหมือนเดิม เพื่อให้หุ่นเดินตรง)
                lat_err = cx - cam_cx 
                
                # 🔥 เปลี่ยนการวาด: วาดเส้นแนวนอนผ่านจุดศูนย์กลางของกล่อง
                cv2.line(annotated_frame, (0, int(cy)), (width, int(cy)), (0, 255, 0), 2)
                
                # วาดจุดกลางจอ
                cv2.circle(annotated_frame, (cam_cx, int(cy)), 5, (0,0,255), -1)
                
                # แสดงค่า Error
                cv2.putText(annotated_frame, f"Lat Err: {int(lat_err)}", (20, 50), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

        self.yolo_lat_error = float(lat_err)
        
        cv2.imshow("Cam 2: YOLO", annotated_frame)
        cv2.waitKey(1)

    def send_packet(self):
        msg_out = Float32MultiArray()
        msg_out.data = [
            float(self.tag_found),
            float(self.tag_error),
            float(self.tag_size),
            float(self.yolo_lat_error),
            float(self.plant_dist),
            float(self.plant_gap),
            float(self.plant_interval)
        ]
        self.data_pub.publish(msg_out)

def main(args=None):
    rclpy.init(args=args)
    node = PCVisionCombined()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()