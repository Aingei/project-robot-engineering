import cv2
import numpy as np
from ultralytics import YOLO

model = YOLO("cabbage2.pt")

# ==========================================
# ส่วนตั้งค่า (ไม่ต้องใช้ .npy)
# ==========================================
CAMERA_HEIGHT = 50.0  # หน่วย cm
# ค่า K นี้หาได้จากการรันครั้งแรกแล้วปรับให้ตรงกับไม้บรรทัด
# หรือคำนวณจาก: (ความกว้างพิกเซล * ระยะห่าง) / ขนาดจริง
PIXEL_CONSTANT = 970.0 

cap = cv2.VideoCapture(1)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret: break

    results = model.track(frame, persist=True)
    annotated = results[0].plot()

    if results[0].boxes is not None:
        for box in results[0].boxes:
            x, y, w, h = box.xywh[0].cpu().numpy()
            cx, cy = int(x), int(y)

            # สำหรับทรงกลม ใช้ค่าเฉลี่ย w, h เพื่อลด Error
            diameter_px = (w + h) / 2
            
            # คำนวณขนาดจริง (cm)
            # สูตร: (พิกเซล * ระยะห่างจริง) / ค่าคงที่กล้อง
            diameter_cm = (diameter_px * CAMERA_HEIGHT) / PIXEL_CONSTANT

            # วาดจุดกึ่งกลางและแสดงขนาด
            cv2.circle(annotated, (cx, cy), 5, (0, 0, 255), -1)
            cv2.putText(annotated, f"D = {diameter_cm:.2f} cm", (cx-70, cy-20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

    cv2.imshow("Sphere Measurement - Manual Constant", annotated)
    if cv2.waitKey(1) & 0xFF == ord("q"): break

cap.release()
cv2.destroyAllWindows()