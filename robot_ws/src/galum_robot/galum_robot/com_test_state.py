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
from rclpy import qos

class PCVisionCombined(Node):
    def __init__(self):
        super().__init__('pc_vision_combined')

        # --- Settings ---
        self.yolo_model = "rack02.pt" 
        self.TAG_REAL_SIZE = 0.07  
        self.camera_params = [600, 600, 320, 240] 

        # Subscriptions
        self.create_subscription(CompressedImage, '/camera/stream', self.process_april, qos_profile=qos.qos_profile_sensor_data)
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
        self.z_dist = 0.0
        
        # 🔥 เพิ่ม: ตัวแปรเช็คว่าเจอแปลงไหม (0=ไม่เจอ, 1=เจอ)
        self.yolo_found = 0.0

        # Image buffers
        self.img_april = None
        self.img_yolo = None

        self.get_logger().info("PC Vision Combined Started (Wait-For-Scan Mode)")

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
        
        detections = self.at_detector.detect(gray, 
                                             estimate_tag_pose=True, 
                                             camera_params=self.camera_params, 
                                             tag_size=self.TAG_REAL_SIZE)
        
        found = 0.0
        if detections:
            tag = max(detections, key=lambda x: x.decision_margin)
            found = 1.0
            self.tag_error = float(center_x - int(tag.center[0]))
            self.tag_size = float(abs(tag.corners[1][0] - tag.corners[0][0]))
            self.z_dist = float(tag.pose_t[2][0]) 
            
            p_dist, gap, interval = self.decode_cabbage_data(tag.tag_id)
            self.plant_dist = float(p_dist)
            self.plant_gap = float(gap)
            self.plant_interval = float(interval)

            cv2.polylines(frame, [tag.corners.astype(int)], True, (0, 255, 0), 2)
            text = f"Dist:{p_dist} Z:{self.z_dist:.2f}m"
            cv2.putText(frame, text, (int(tag.center[0]), int(tag.center[1])), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        self.tag_found = found
        self.img_april = frame

    def process_yolo(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None: return

        height, width = frame.shape[:2]
        cam_cx = width // 2
        
        results = self.model.track(frame, persist=True, device=0, imgsz=320, verbose=False)
        annotated_frame = results[0].plot()
        
        lat_err = 0.0
        found = 0.0 # 🔥 เริ่มต้นเป็น 0 (ไม่เจอ)

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
                lat_err = cx - cam_cx 
                found = 1.0 # 🔥 เจอแปลงแล้ว!
                
                cv2.line(annotated_frame, (0, int(cy)), (width, int(cy)), (0, 255, 0), 2)
                cv2.circle(annotated_frame, (cam_cx, int(cy)), 5, (0,0,255), -1)

        self.yolo_lat_error = float(lat_err)
        self.yolo_found = float(found) # 🔥 อัพเดทตัวแปร
        
        self.img_yolo = annotated_frame
    
    def resize_for_stack(self, img, target_h=480):
        h, w = img.shape[:2]
        ratio = target_h / float(h)
        target_w = int(w * ratio)
        return cv2.resize(img, (target_w, target_h))

    def send_packet(self):
        msg_out = Float32MultiArray()
        msg_out.data = [
            float(self.tag_found),
            float(self.tag_error),
            float(self.tag_size),
            float(self.yolo_lat_error),
            float(self.plant_dist),
            float(self.plant_gap),
            float(self.plant_interval),
            float(self.z_dist),
            float(self.yolo_found)  # 🔥 เพิ่มตัวที่ 9 (Index 8): บอกหุ่นว่าเจอแปลงไหม
        ]
        self.data_pub.publish(msg_out)
        
        # Display Combined View
        if (self.img_april is not None) and (self.img_yolo is not None):
            v1 = self.resize_for_stack(self.img_april, 480)
            v2 = self.resize_for_stack(self.img_yolo, 480)
            combined = np.hstack((v1, v2))
            cv2.imshow("Galum Robot Vision", combined)
            cv2.waitKey(1)
        elif self.img_april is not None:
             cv2.imshow("Galum Robot Vision", self.img_april)
             cv2.waitKey(1)

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