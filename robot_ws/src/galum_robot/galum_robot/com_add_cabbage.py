#!/usr/bin/env python3
"""
pc_vision_combined.py
(YOLO Lane Tracking + AprilTag Front + Back Scan Mode + Cabbage Scan Mode)
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

class KalmanTracker:
    def __init__(self, process_noise: float = 0.1, meas_noise: float = 0.1):
        self.kf = cv2.KalmanFilter(4, 2)
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
    CAMERA_PARAMS    = [600, 600, 160, 120]   # Auto-updated
    YOLO_MODEL       = "rack_segm02.pt"
    CABBAGE_MODEL    = "cabbage2.pt"          # [NEW] โมเดลวัดขนาดกะหล่ำ
    MORPH_KERNEL     = 9
    DISPLAY_H        = 240
    LOOK_AHEAD_RATIO = 0.7
    
    # [NEW] พารามิเตอร์สำหรับกล้องวัดขนาดกะหล่ำ (กล้องล่าง)
    CAMERA_HEIGHT    = 50.0  # หน่วย cm 
    PIXEL_CONSTANT   = 485.0 
    # ────────────────────────────────────────────────────────

    def __init__(self):
        super().__init__('pc_vision_combined')

        self.at_detector = pupil_apriltags.Detector(families='tagStandard52h13')
        
        self.kalman = KalmanTracker(process_noise=0.1, meas_noise=0.1)
        
        self.get_logger().info("Loading YOLO Models...")
        self.model = YOLO(self.YOLO_MODEL)
        self.model_cabbage = YOLO(self.CABBAGE_MODEL)

        sub_qos = qos.qos_profile_sensor_data
        self.create_subscription(CompressedImage, '/camera/stream',      self.cb_april, sub_qos)
        self.create_subscription(CompressedImage, '/camera/stream/plot', self.cb_yolo,  sub_qos)

        # scan_mode: 0.0=YOLO Lane, 1.0=Back AprilTag, 2.0=Cabbage Scan
        self.create_subscription(Float32, '/galum/scan_mode', self.cb_scan_mode, sub_qos)

        self.data_pub = self.create_publisher(Float32MultiArray, '/galum/vision_packet', sub_qos)
        self.create_timer(0.05, self.send_packet)
        # 🟢 [เพิ่มบรรทัดนี้] รอรับคำสั่งให้เซฟรูป
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
        self.cabbage_diameter_cm = 0.0 # [NEW] ขนาดกะหล่ำ

    def cb_scan_mode(self, msg):
        self.scan_mode = msg.data
        
    def cb_save_image(self, msg):
        filename = msg.data # ดึงชื่อไฟล์ที่ master ส่งมาให้
        if self.img_yolo is not None:
            # ใช้ cv2.imwrite เพื่อเซฟรูปลงเครื่อง
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

    # ── YOLO + BACK SCAN + CABBAGE SCAN (กล้องล่าง /camera/stream/plot) ──
    def cb_yolo(self, msg):
        frame = self._decode(msg)
        if frame is None: return
        h, w = frame.shape[:2]
        cam_cx = w // 2

        # [NEW] สลับโหมดกล้องล่างตาม scan_mode
        if self.scan_mode == 1.0:
            self._cb_back_scan(frame, h, w)
            return
        elif self.scan_mode == 2.0:
            self._cb_cabbage_scan(frame, h, w)
            return

        # ── โหมด 0.0: YOLO Lane Tracking ──
        results = self.model.track(frame, persist=True, device=0, imgsz=320, verbose=False)

        found = False
        target_cx = float(cam_cx)
        angle_deg = 0.0
        mode = 0.0

        cv2.line(frame, (cam_cx, 0), (cam_cx, h), (100, 100, 100), 1)

        if results[0].masks is not None:
            mask = self._build_clean_mask(results[0].masks.data.cpu().numpy(), h, w)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                c = max(contours, key=cv2.contourArea)
                M = cv2.moments(c)
                if M["m00"] > 0:
                    raw_cx = int(M["m10"] / M["m00"])
                    raw_cy = int(M["m01"] / M["m00"])
                else:
                    rect = cv2.minAreaRect(c)
                    raw_cx, raw_cy = int(rect[0][0]), int(rect[0][1])

                line = cv2.fitLine(c, cv2.DIST_L2, 0, 0.01, 0.01)
                vx, vy, x0, y0 = line.flatten()
                vx, vy, x0, y0 = float(vx), float(vy), float(x0), float(y0)

                look_ahead_y = int(h * self.LOOK_AHEAD_RATIO)
                bottom_mask = mask[h//2:, :]
                bottom_contours, _ = cv2.findContours(
                    bottom_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

                if bottom_contours:
                    bc = max(bottom_contours, key=cv2.contourArea)
                    bM = cv2.moments(bc)
                    if bM["m00"] > 0:
                        target_cx = float(bM["m10"] / bM["m00"])
                        look_ahead_y = h//2 + int(bM["m01"] / bM["m00"])
                    else:
                        target_cx = float(np.clip(x0, 0, w))
                else:
                    if abs(vy) > 1e-4:
                        calculated_target_x = x0 + (look_ahead_y - y0) * (vx / vy)
                        target_cx = float(np.clip(calculated_target_x, 0, w))
                    else:
                        target_cx = float(x0)

                kx, ky = self.kalman.update(target_cx, look_ahead_y)
                target_cx = float(kx)

                raw_angle = math.degrees(math.atan2(vy, vx))
                if abs(raw_angle) > 90:
                    raw_angle = raw_angle - math.copysign(180, raw_angle)
                angle_deg = raw_angle

                if abs(vy) > 1e-4:
                    x_top = int(x0 + (0 - y0) * (vx / vy))
                    x_bot = int(x0 + (h - y0) * (vx / vy))
                    cv2.line(frame, (x_top, 0), (x_bot, h), (255, 0, 0), 2)
                else:
                    cv2.line(frame, (0, int(y0)), (w, int(y0)), (255, 0, 0), 2)

                cv2.circle(frame, (int(target_cx), look_ahead_y), 8, (0, 255, 0), -1)
                found = True
                mode = 2.0

        if not found and results[0].boxes is not None and len(results[0].boxes) > 0:
            best_box = min(results[0].boxes, key=lambda b: abs(b.xywh[0][0].item() - cam_cx))
            bx, by, bw, bh = best_box.xywh[0].tolist()
            kx, ky = self.kalman.update(bx, by)
            target_cx = float(kx)

            x1, y1, x2, y2 = best_box.xyxy[0].tolist()
            cv2.rectangle(frame, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 255), 2)
            cv2.circle(frame, (int(kx), int(ky)), 8, (0, 255, 0), -1)
            found = True
            mode = 1.0

        if not found and self.kalman.initialized:
            px, py = self.kalman.predict_only()
            if px is not None:
                target_cx = float(px)
                cv2.circle(frame, (px, py), 6, (255, 0, 255), -1)

        cx_error = float(target_cx) - float(cam_cx)

        self.yolo_found    = 1.0 if found else 0.0
        self.lane_center_x = cx_error
        self.heading_angle = angle_deg
        self.detect_mode   = mode
        self.kalman_valid  = 1.0 if self.kalman.initialized else 0.0

        info = f"M:{int(mode)} Err:{cx_error:.0f} A:{angle_deg:.1f}"
        cv2.putText(frame, info, (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
        self.img_yolo = frame

    def _cb_back_scan(self, frame, h, w):
        """โหมด 1.0: ใช้กล้อง YOLO ถอยหลังหา AprilTag"""
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
            float(self.lane_center_x),    # [3]
            float(self.plant_dist),       # [4]
            float(self.plant_gap),        # [5]
            float(self.plant_interval),   # [6]
            float(self.z_dist),           # [7]
            float(self.yolo_found),       # [8]
            float(self.heading_angle),    # [9]
            float(self.detect_mode),      # [10]
            float(self.kalman_valid),     # [11]
            float(self.back_tag_found),   # [12]
            float(self.back_tag_y_ratio), # [13]
            float(self.cabbage_diameter_cm) # [14] [NEW] ส่งขนาดกะหล่ำกลับ
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
        rclpy.shutdown()

if __name__ == '__main__':
    main()