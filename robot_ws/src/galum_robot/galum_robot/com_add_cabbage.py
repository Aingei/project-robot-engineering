#!/usr/bin/env python3
"""
pc_vision_combined.py
(AprilTag Front + Back Scan Mode + Cabbage Scan Mode)
* Removed YOLO Lane Tracking to fix performance lag *
"""

import cv2
import numpy as np
import math
import pupil_apriltags
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Float32MultiArray, Float32 , String
from ultralytics import YOLO
from rclpy import qos

class PCVisionCombined(Node):
    # ── TUNING ──────────────────────────────────────────────
    TAG_REAL_SIZE    = 0.07
    CAMERA_PARAMS    = [600, 600, 160, 120]   # Auto-updated
    CABBAGE_MODEL    = "cabbage2.pt"          # โมเดลวัดขนาดกะหล่ำ
    DISPLAY_H        = 240
    
    # พารามิเตอร์สำหรับกล้องวัดขนาดกะหล่ำ (กล้องล่าง)
    CAMERA_HEIGHT    = 50.0  # หน่วย cm 
    PIXEL_CONSTANT   = 480.0 # 485 
    # ────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('pc_vision_combined')

        self.at_detector = pupil_apriltags.Detector(families='tagStandard52h13')
        
        self.get_logger().info("Loading Cabbage YOLO Model only...")
        # โหลดแค่โมเดลกะหล่ำ ไม่โหลดโมเดล Lane Tracking แล้วเพื่อลดอาการแลค
        self.model_cabbage = YOLO(self.CABBAGE_MODEL)

        sub_qos = qos.qos_profile_sensor_data
        self.create_subscription(CompressedImage, '/camera/stream',      self.cb_april, sub_qos)
        self.create_subscription(CompressedImage, '/camera/stream/plot', self.cb_yolo,  sub_qos)

        # scan_mode: 0.0=Idle, 1.0=Back AprilTag, 2.0=Cabbage Scan
        self.create_subscription(Float32, '/galum/scan_mode', self.cb_scan_mode, sub_qos)

        self.data_pub = self.create_publisher(Float32MultiArray, '/galum/vision_packet', sub_qos)
        self.create_timer(0.05, self.send_packet)
        self.create_subscription(String, '/galum/save_image_cmd', self.cb_save_image, 10)

        # Vars
        self.tag_found=0.0; self.tag_error=0.0; self.tag_size=0.0
        self.plant_dist=0.0; self.plant_gap=0.0; self.plant_interval=0.0; self.z_dist=0.0
        self.yolo_found=0.0; self.lane_center_x=0.0; self.heading_angle=0.0
        self.detect_mode=0.0; self.kalman_valid=0.0
        self.img_april=None; self.img_yolo=None

        self.scan_mode = 0.0   
        self.back_tag_found = 0.0   
        self.back_tag_y_ratio = 1.0 
        self.cabbage_diameter_cm = 0.0

    def cb_scan_mode(self, msg):
        self.scan_mode = msg.data
        
    def cb_save_image(self, msg):
        filename = msg.data
        if self.img_yolo is not None:
            cv2.imwrite(filename, self.img_yolo)
            self.get_logger().info(f"📸 แชะ! บันทึกภาพกะหล่ำลงไฟล์: {filename} เรียบร้อยแล้ว")

    # ── APRILTAG (กล้องหน้า/กล้องบน) ────────────────────────────────
    def cb_april(self, msg):
        frame = self._decode(msg)
        if frame is None: return
        fh, fw = frame.shape[:2]
        cx_img = fw // 2

        self.CAMERA_PARAMS = [600, 600, fw // 2, fh // 2]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.at_detector.detect(
            gray, estimate_tag_pose=True,
            camera_params=self.CAMERA_PARAMS,
            tag_size=self.TAG_REAL_SIZE)

        if detections:
            tag = max(detections, key=lambda d: d.decision_margin)
            self.tag_found = 1.0
            self.tag_error = float(cx_img - int(tag.center[0]))
            self.tag_size  = float(abs(tag.corners[1][0] - tag.corners[0][0]))
            self.z_dist    = float(tag.pose_t[2][0])
            p, g, iv = self._decode_tag(tag.tag_id)
            self.plant_dist, self.plant_gap, self.plant_interval = float(p), float(g), float(iv)

            pts = tag.corners.astype(int)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

            debug_text = f"{tag.tag_id} = {p},{g},{iv}"
            z_text     = f"Z: {self.z_dist:.2f}m"
            tx, ty = int(tag.center[0]), int(tag.center[1])
            cv2.putText(frame, debug_text, (tx - 60, ty - 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
            cv2.putText(frame, z_text,     (tx - 30, ty + 30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 200, 0), 1)
        else:
            self.tag_found = 0.0

        cv2.putText(frame, f"{fw}x{fh} cx:{fw//2} cy:{fh//2}",
                    (5, fh - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.35, (255, 255, 0), 1)

        self.img_april = frame

    def _decode_tag(self, tag_id):
        s = str(tag_id).zfill(5)
        if len(s) != 5: return 0, 0, 0
        gap = {'1':5,'2':10,'3':15,'4':20,'5':25}.get(s[2], 0)
        return int(s[0:2]), gap, int(s[3:5])

    # ── กล้องล่าง (/camera/stream/plot) ──
    def cb_yolo(self, msg):
        frame = self._decode(msg)
        if frame is None: return
        h, w = frame.shape[:2]

        # สลับโหมดกล้องล่างตาม scan_mode
        if self.scan_mode == 1.0:
            self._cb_back_scan(frame, h, w)
        elif self.scan_mode == 2.0:
            self._cb_cabbage_scan(frame, h, w)
        else:
            # โหมด 0.0: เอา Lane Tracking ออกแล้ว ให้คืนค่าเป็น 0.0 เพื่อลดการใช้ CPU/GPU
            self.yolo_found = 0.0
            self.lane_center_x = 0.0
            self.heading_angle = 0.0
            self.detect_mode = 0.0
            self.kalman_valid = 0.0
            cv2.putText(frame, "IDLE (Lane Tracking Disabled)", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            self.img_yolo = frame

    def _cb_back_scan(self, frame, h, w):
        """โหมด 1.0: ถอยหลังหา AprilTag"""
        fh, fw = frame.shape[:2]
        cam_params = [600, 600, fw // 2, fh // 2]

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.at_detector.detect(
            gray, estimate_tag_pose=True,
            camera_params=cam_params,
            tag_size=self.TAG_REAL_SIZE)

        threshold_y = int(h * 0.2)
        cv2.line(frame, (0, threshold_y), (w, threshold_y), (0, 255, 255), 1)
        cv2.putText(frame, "BACK SCAN MODE", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

        if detections:
            tag = max(detections, key=lambda d: d.decision_margin)
            ty_center = float(tag.center[1])
            self.back_tag_found   = 1.0
            self.back_tag_y_ratio = ty_center / float(h) 

            pts = tag.corners.astype(int)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            cv2.putText(frame, f"Y_ratio:{self.back_tag_y_ratio:.2f}",
                        (int(tag.center[0]) - 40, int(tag.center[1]) - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
        else:
            self.back_tag_found   = 0.0
            self.back_tag_y_ratio = 1.0 

        self.img_yolo = frame

    def _cb_cabbage_scan(self, frame, h, w):
        """โหมด 2.0: ใช้กล้อง YOLO วัดขนาดกะหล่ำ"""
        results = self.model_cabbage.track(frame, persist=True, verbose=False)
        
        cv2.putText(frame, "CABBAGE MEASUREMENT", (10, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        largest_diameter = 0.0

        if results[0].boxes is not None and len(results[0].boxes) > 0:
            for box in results[0].boxes:
                x, y, bw, bh = box.xywh[0].cpu().numpy()
                cx, cy = int(x), int(y)

                diameter_px = (bw + bh) / 2
                diameter_cm = (diameter_px * self.CAMERA_HEIGHT) / self.PIXEL_CONSTANT
                
                if diameter_cm > largest_diameter:
                    largest_diameter = float(diameter_cm)

                cv2.circle(frame, (cx, cy), 5, (0, 0, 255), -1)
                cv2.putText(frame, f"D={diameter_cm:.2f}cm", (cx-40, cy-20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

        self.cabbage_diameter_cm = largest_diameter
        self.img_yolo = frame

    # ── UTILS ───────────────────────────────────────────────
    def _decode(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    def _resize_h(self, img, target_h=240):
        if img is None: return None
        h, w = img.shape[:2]
        return cv2.resize(img, (int(w * target_h / h), target_h))

    def send_packet(self):
        msg = Float32MultiArray()
        msg.data = [
            float(self.tag_found),        # [0]
            float(self.tag_error),        # [1]
            float(self.tag_size),         # [2]
            float(self.lane_center_x),    # [3] (Will be 0.0)
            float(self.plant_dist),       # [4]
            float(self.plant_gap),        # [5]
            float(self.plant_interval),   # [6]
            float(self.z_dist),           # [7]
            float(self.yolo_found),       # [8] (Will be 0.0)
            float(self.heading_angle),    # [9] (Will be 0.0)
            float(self.detect_mode),      # [10] (Will be 0.0)
            float(self.kalman_valid),     # [11] (Will be 0.0)
            float(self.back_tag_found),   # [12]
            float(self.back_tag_y_ratio), # [13]
            float(self.cabbage_diameter_cm) # [14]
        ]
        self.data_pub.publish(msg)

        if self.img_april is not None and self.img_yolo is not None:
            v1 = self._resize_h(self.img_april, self.DISPLAY_H)
            v2 = self._resize_h(self.img_yolo, self.DISPLAY_H)
            if v1 is not None and v2 is not None:
                cv2.imshow("Galum Vision", np.hstack((v1, v2)))
                cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = PCVisionCombined()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        cv2.destroyAllWindows()
        cv2.waitKey(1)
        rclpy.shutdown()

if __name__ == '__main__':
    main()