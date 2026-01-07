import cv2
import time

cap = cv2.VideoCapture(0)

while True:
    t = time.time()
    cap.read()

    print(time.time()-t)
