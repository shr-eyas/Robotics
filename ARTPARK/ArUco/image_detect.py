import cv2

image = cv2.imread('1.png')

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
parameters =  cv2.aruco.DetectorParameters_create()

corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

print(ids)
cv2.aruco.drawDetectedMarkers(image,corners,ids)

while True:
    if cv2.waitKey(1)==113:
        break
    cv2.imshow('image',image)
