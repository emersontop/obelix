#!/usr/bin/env python3
import cv2

webcam = cv2.VideoCapture(0)

print("Vamos comecar")

try:
    if webcam.isOpened():
        validacao, frame = webcam.read()
        while validacao:
            validacao, frame = webcam.read()
            #print(frame)
            #frame = cv2.rotate(frame, cv2.ROTATE_180)
            cv2.imshow("Camera log",frame)
            key = cv2.waitKey(2)
except KeyboardInterrupt:
    webcam.release()
    print('FIM')