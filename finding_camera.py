import cv2
import numpy as np
# Test Camera 1 with MSMF
print(cv2.__version__)

cap1 = cv2.VideoCapture(1) 

cap2 = cv2.VideoCapture(3)  # For Windows MSMF
while True:
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()
        cv2.imshow('Camera 1', frame1)
        cv2.imshow('Camera 2', frame2)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
