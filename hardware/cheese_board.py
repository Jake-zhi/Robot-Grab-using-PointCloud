import cv2
import numpy as np

width = 1200
height = 1800
length = 150

image = np.zeros((width,height),dtype = np.uint8)
print(image.shape[0],image.shape[1])

for j in range(height):
    for i in range(width):
        if( int(i/length) + int(j/length))%2:
            image[i, j] = 255
cv2.imwrite("chess.jpg", image)
cv2.imshow("chess", image)
cv2.waitKey(0)
