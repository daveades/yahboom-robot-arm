import cv2
import numpy as np

cols, rows = 9, 7      # squares
sq = 80                # pixels per square

img = np.zeros((rows * sq, cols * sq), dtype=np.uint8)

for r in range(rows):
    for c in range(cols):
        if (r + c) % 2 == 0:
            img[r * sq:(r + 1) * sq, c * sq:(c + 1) * sq] = 255

cv2.imwrite("checkerboard_9x7.png", img)
print("Saved checkerboard_9x7.png")

