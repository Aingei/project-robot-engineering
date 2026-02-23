from flask import Flask, Response
import cv2

app = Flask(__name__)

# เปิดกล้อง C922 (Plug & Play)
cap = cv2.VideoCapture(0)  # 0 = /dev/video0
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

def gen_frames():
    while True:
        success, frame = cap.read()
        if not success:
            continue
        else:
            ret, buffer = cv2.imencode('.jpg', frame)
            frame = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame + b'\r\n')

@app.route('/video')
def video():
    return Response(gen_frames(),
                    mimetype='multipart/x-mixed-replace; boundary=frame')

if __name__ == "__main__":
    app.run(host='0.0.0.0', port=5000)
