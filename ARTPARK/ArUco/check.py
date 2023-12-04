# code to check if the path of the file is correct
import cv2
import sys

img = cv2.imread('ARTPARK/1.png')
if img is None:
    print("Error: Unable to load image. Check the file path.")
    sys.exit(1)  # or exit(1)
