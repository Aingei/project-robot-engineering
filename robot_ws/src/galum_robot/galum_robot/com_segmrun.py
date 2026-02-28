#!/usr/bin/env python3
"""
pc_vision_combined.py
(Fixed: ZeroDivisionError + fitLine flattening)
"""

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

class KalmanTracker:
    def __init__(self, process_noise: float = 0.1, meas_noise: float = 0.1):
        self.kf = cv2.KalmanFilter(4, 2)
        # State: [x, y, vx, vy]
        self.kf.transitionMatrix    = np.array([[1,0,1,0],[0,1,0,1],[0,0,1,0],[0,0,0,1]], np.float32)
        self.kf.measurementMatrix   = np.array([[1,0,0,0],[0,1,0,0]], np.float32)
        self.kf.processNoiseCov     = np.eye(4, dtype=np.float32) * process_noise
        self.kf.measurementNoiseCov = np.eye(2, dtype=np.float32) * meas_noise
        self.kf.errorCovPost        = np.eye(4, dtype=np.float32)
        self.initialized = False

    def update(self, cx: float, cy: float):
        if not self.initialized:
            self.kf.statePost = np.array([[cx],[cy],[0],[0]], np.float32)
            self.initialized = True
        self.kf.predict()
        est = self.kf.correct(np.array([[np.float32(cx)],[np.float32(cy)]]))
        return int(est[0][0]), int(est[1][0])

    def predict_only(self):
        if not self.initialized: return None, None
        pred = self.kf.predict()
        return int(pred[0][0]), int(pred[1][0])

class PCVisionCombined(Node):
    # ── TUNING ──────────────────────────────────────────────
    TAG_REAL_SIZE    = 0.07
    CAMERA_PARAMS    = [600, 600, 320, 240]
    YOLO_MODEL       = "rack_segm.pt"
    MORPH_KERNEL     = 9
    DISPLAY_H        = 480
    LOOK_AHEAD_RATIO = 0.7 
    # ────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('pc_vision_combined')
        
        self.at_detector = pupil_apriltags.Detector(families='tagStandard52h13')
        self.get_logger().info("Loading YOLO (Safe Draw)...")
        self.model = YOLO(self.YOLO_MODEL)
        self.kalman = KalmanTracker(process_noise=0.1, meas_noise=0.1)

        sub_qos = qos.qos_profile_sensor_data
        self.create_subscription(CompressedImage, '/camera/stream',      self.cb_april, sub_qos)
        self.create_subscription(CompressedImage, '/camera/stream/plot', self.cb_yolo,  sub_qos)
        self.data_pub = self.create_publisher(Float32MultiArray, '/galum/vision_packet', sub_qos)
        self.create_timer(0.05, self.send_packet)

        # Vars
        self.tag_found=0.0; self.tag_error=0.0; self.tag_size=0.0
        self.plant_dist=0.0; self.plant_gap=0.0; self.plant_interval=0.0; self.z_dist=0.0
        self.yolo_found=0.0; self.lane_center_x=0.0; self.heading_angle=0.0
        self.detect_mode=0.0; self.kalman_valid=0.0
        self.img_april=None; self.img_yolo=None

    # ── APRILTAG ────────────────────────────────────────────
    def cb_april(self, msg):
        frame = self._decode(msg)
        if frame is None: return
        cx_img = frame.shape[1] // 2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        detections = self.at_detector.detect(gray, estimate_tag_pose=True, camera_params=self.CAMERA_PARAMS, tag_size=self.TAG_REAL_SIZE)
        
        if detections:
            tag = max(detections, key=lambda d: d.decision_margin)
            self.tag_found = 1.0
            self.tag_error = float(cx_img - int(tag.center[0]))
            self.tag_size  = float(abs(tag.corners[1][0] - tag.corners[0][0]))
            self.z_dist    = float(tag.pose_t[2][0])
            p, g, iv = self._decode_tag(tag.tag_id)
            self.plant_dist, self.plant_gap, self.plant_interval = float(p), float(g), float(iv)
            pts = tag.corners.astype(int)
            cv2.polylines(frame, [pts], True, (0,255,0), 2)
        else:
            self.tag_found = 0.0
        self.img_april = frame

    def _decode_tag(self, tag_id):
        s = str(tag_id).zfill(5)
        if len(s)!=5: return 0,0,0
        gap = {'1':5,'2':10,'3':15,'4':20,'5':25}.get(s[2], 0)
        return int(s[0:2]), gap, int(s[3:5])

    # ── YOLO (Safe Math) ────────────────────────────────────
    def cb_yolo(self, msg):
        frame = self._decode(msg)
        if frame is None: return
        h, w = frame.shape[:2]
        cam_cx = w // 2

        results = self.model.track(frame, persist=True, device=0, imgsz=320, verbose=False)

        found = False
        target_cx = float(cam_cx) 
        angle_deg = 0.0
        mode = 0.0
        
        cv2.line(frame, (cam_cx, 0), (cam_cx, h), (100,100,100), 1)

        # --- SEGMENTATION ---
        if results[0].masks is not None:
            mask = self._build_clean_mask(results[0].masks.data.cpu().numpy(), h, w)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            if contours:
                c = max(contours, key=cv2.contourArea)
                
                # 1. Centroid
                M = cv2.moments(c)
                if M["m00"] > 0:
                    raw_cx = int(M["m10"] / M["m00"])
                    raw_cy = int(M["m01"] / M["m00"])
                else:
                    rect = cv2.minAreaRect(c)
                    raw_cx, raw_cy = int(rect[0][0]), int(rect[0][1])

                # 2. fitLine (Flatten first)
                line = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
                vx, vy, x0, y0 = line.flatten()
                vx, vy, x0, y0 = float(vx), float(vy), float(x0), float(y0)

                # 3. Calculate Target (Prevent Divide by Zero)
                look_ahead_y = int(h * self.LOOK_AHEAD_RATIO)
                
                # Safe division check
                if abs(vy) > 1e-4:
                    # x = x0 + (y - y0) * (vx / vy)
                    calculated_target_x = x0 + (look_ahead_y - y0) * (vx / vy)
                    target_cx = float(calculated_target_x)
                else:
                    target_cx = float(x0) # เส้นนอน เล็งที่จุดกลางเลย

                kx, ky = self.kalman.update(target_cx, look_ahead_y)
                target_cx = float(kx)

                # 4. Angle
                angle_deg = math.degrees(math.atan2(vy, vx))

                # 5. Draw Infinite Line (Safe Logic)
                if abs(vy) > 1e-4:
                    # วาดจากบนสุด (y=0) ถึงล่างสุด (y=h)
                    x_top = int(x0 + (0 - y0) * (vx / vy))
                    x_bot = int(x0 + (h - y0) * (vx / vy))
                    cv2.line(frame, (x_top, 0), (x_bot, h), (255, 0, 0), 2)
                else:
                    # เส้นแนวนอน วาดซ้ายไปขวา
                    cv2.line(frame, (0, int(y0)), (w, int(y0)), (255, 0, 0), 2)

                cv2.circle(frame, (int(target_cx), look_ahead_y), 8, (0, 255, 0), -1)
                found = True
                mode = 2.0

        # --- BOX FALLBACK ---
        if not found and results[0].boxes is not None and len(results[0].boxes) > 0:
            best_box = min(results[0].boxes, key=lambda b: abs(b.xywh[0][0].item() - cam_cx))
            bx, by, bw, bh = best_box.xywh[0].tolist()
            kx, ky = self.kalman.update(bx, by)
            target_cx = float(kx)
            
            x1, y1, x2, y2 = best_box.xyxy[0].tolist()
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0,255,255), 2)
            cv2.circle(frame, (int(kx), int(ky)), 8, (0,255,0), -1)
            found = True
            mode = 1.0

        # --- PREDICT ---
        if not found and self.kalman.initialized:
            px, py = self.kalman.predict_only()
            if px is not None:
                target_cx = float(px)
                cv2.circle(frame, (px, py), 6, (255,0,255), -1)

        # Output Calc (Target - Cam เพื่อดึงเข้ากลาง)
        cx_error = float(target_cx) - float(cam_cx)

        self.yolo_found = 1.0 if found else 0.0
        self.lane_center_x = cx_error
        self.heading_angle = angle_deg
        self.detect_mode = mode
        self.kalman_valid = 1.0 if self.kalman.initialized else 0.0

        info = f"M:{int(mode)} Err:{cx_error:.0f} A:{angle_deg:.1f}"
        cv2.putText(frame, info, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        self.img_yolo = frame

    # ── UTILS ───────────────────────────────────────────────
    def _decode(self, msg):
        arr = np.frombuffer(msg.data, np.uint8)
        return cv2.imdecode(arr, cv2.IMREAD_COLOR)

    def _build_clean_mask(self, masks, h, w):
        combined = np.zeros((h, w), dtype=np.uint8)
        for m in masks:
            m8 = (m * 255).astype(np.uint8)
            m8 = cv2.resize(m8, (w, h))
            combined = cv2.bitwise_or(combined, m8)
        combined = cv2.medianBlur(combined, 5)
        k = np.ones((self.MORPH_KERNEL, self.MORPH_KERNEL), np.uint8)
        combined = cv2.morphologyEx(combined, cv2.MORPH_CLOSE, k)
        return cv2.morphologyEx(combined, cv2.MORPH_OPEN, k)

    def _resize_h(self, img, target_h=480):
        if img is None: return None
        h, w = img.shape[:2]
        return cv2.resize(img, (int(w * target_h / h), target_h))

    def send_packet(self):
        msg = Float32MultiArray()
        msg.data = [
            float(self.tag_found), float(self.tag_error), float(self.tag_size),
            float(self.lane_center_x), float(self.plant_dist), float(self.plant_gap),
            float(self.plant_interval), float(self.z_dist), float(self.yolo_found),
            float(self.heading_angle), float(self.detect_mode), float(self.kalman_valid)
        ]
        self.data_pub.publish(msg)
        
        if self.img_april is not None and self.img_yolo is not None:
            v1 = self._resize_h(self.img_april, self.DISPLAY_H)
            v2 = self._resize_h(self.img_yolo, self.DISPLAY_H)
            if v1 is not None and v2 is not None:
                cv2.imshow("Galum Vision (Safe)", np.hstack((v1, v2)))
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()