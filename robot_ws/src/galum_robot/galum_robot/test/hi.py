# import pupil_apriltags
# print("OK")

# def main():
#     print("Test node running")

import cv2
def main():
    print("Opening camera...")
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    print("Read:", ret)
    cap.release()
    