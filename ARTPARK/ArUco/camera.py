import cv2
import cv2.aruco as arcuo

VideoCap = False
cap = cv2.VideoCapture(0)

while True:
    _,img=cap.read()
    if cv2.waitKey(1)==113:
        break
    cv2.imshow('img',img)
