import cv2
import pupil_apriltags

# ==========================================
# 1. ตั้งค่า (Config)
# ถ้าจะเอาไปใส่หุ่นยนต์วิ่งจริงแล้วไม่อยต่อจอ ให้แก้เป็น False
SHOW_DEBUG_WINDOW = True  

detector = pupil_apriltags.Detector(families='tagStandard52h13')
cap = cv2.VideoCapture(1)

# ==========================================
# 2. Logic
def decode_cabbage_data(tag_id):
    
    # เปลี่ยนเลข ID เป็นสตริงและเติมเลข 0 ข้างหน้าให้ครบ 5 หลัก (เช่น ID 123 กลายเป็น "00123")
    s = str(tag_id).zfill(5)
    if len(s) != 5: return None # ถ้าเลขมั่ว ให้ส่งค่าว่างกลับไป

    # ถอดรหัสตามคู่มือ
    data = {
        "id": tag_id,
        "planting_dist": int(s[0:2]),      # AB
        "gap": {'1':5, '2':10, '3':15 , '4':20 , '5':25}.get(s[2], 0), # C (Mapping)
        # .get(key, default)  
        # ถ้าเกิดเหตุการณ์ที่อ่าน Tag มาแล้วเลขหลักที่ 3 ไม่ใช่ 1-5 (เช่น อ่านได้เลข 9 ซึ่งไม่มีในตาราง) โปรแกรมจะไม่ค้าง แต่จะส่งค่า 0
        "interval": int(s[3:5])            # DE
    }
    return data

# ==========================================
# 3. ลูปหลัก (Main Loop)
while True:
    ret, frame = cap.read()
    if not ret: break
    
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # แปลงภาพจากสี (BGR) เป็น ขาวดำ (Gray)
    detections = detector.detect(gray) # สั่งให้ Detector ค้นหา Tag ในภาพ

    # processing
    if detections: #เจอ 1 อัน
        tag = detections[0] # สมมติเอา Tag แรกที่เจอมาใช้
        result = decode_cabbage_data(tag.tag_id)
        
        if result:
            
            # พิมพ์ค่าที่ถอดรหัสได้ออกมาทางหน้าจอ Console
            print(f"move: distance {result['planting_dist']} | เว้น {result['gap']} | {result['interval']}")
            
            # ตัวอย่าง: ส่งค่าเข้าฟังก์ชันควบคุมหุ่น
            # robot_controller.move(result['planting_dist']) 
            # ros_publisher.publish(str(result))
            
    # show
    if SHOW_DEBUG_WINDOW:
        for tag in detections:
            # วาดกรอบสี่เหลี่ยม
            pts = tag.corners.reshape((-1, 1, 2)).astype(int)
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)
            
            # เขียนตัวเลขบอกบนจอ
            cv2.putText(frame, f"ID: {tag.tag_id}", 
                       (pts[0][0][0], pts[0][0][1]-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        cv2.imshow('Camera View', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

cap.release()
cv2.destroyAllWindows()