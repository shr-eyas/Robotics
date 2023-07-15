# opening live video from phone in openCV
import cv2

#video capture object
cap=cv2.VideoCapture(1) #phone camera

# capture the frames..
while True:
    ret, frame = cap.read()

    cv2.imshow('Frame', frame)  # Display the resulting frame
    key = cv2.waitKey(1)
    if key == 27:  # click esc key to exit
        break

cap.release()
cv2.destroyAllWindows()
