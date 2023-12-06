import cv2
import numpy as np

image = cv2.imread('C:/Users/shrey/Documents/ARTPARK/Line Detection/sudoku.png')

gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
canny = cv2.Canny(gray, 50, 200)

lines = cv2.HoughLines(canny, 1, np.pi/180, 150, np.array([]))


for line in lines:
    rho, theta = line[0]
    a = np.cos(theta)
    b = np.sin(theta)

    x0 = a*rho
    y0 = b*rho

    x1 = int(x0 + 1000*(-b))
    y1 = int(y0 + 1000*(a))
    x2 = int(x0 - 1000*(-b))
    y2 = int(y0 - 1000*(1))

    cv2.line(image, (x1,y1), (x2,y2), (0,255,0), 2)

cv2.imshow('lines', image)
cv2.imshow('canny', canny)

cv2.waitKey(0)
cv2.destroyAllWindows