# code to capture either the image or the video
import cv2
import cv2.aruco as arcuo

VideoCap = False
cap = cv2.VideoCapture(0)

while True:

    if VideoCap: 
        _,img=cap.read()
    else: 
        img = cv2.imread('ARTPARK/ArUco/1.png')
        img = cv2.resize(img,(0,0),fx=0.5,fy=0.5)
        
    if cv2.waitKey(1)==113:
        break   

    cv2.imshow('img',img)
