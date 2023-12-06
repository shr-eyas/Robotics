import cv2

image = cv2.imread('C:/Users/shrey/Documents/ARTPARK/Line Detection/tree.png')

cv2.imwrite('canny.png', cv2.Canny(image, 200, 300))

cv2.imshow('original', image)
cv2.imshow('canny', cv2.imread('canny.png'))

cv2.waitKey(0)
cv2.destroyAllWindows
