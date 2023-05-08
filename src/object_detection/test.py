import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

img = cv.imread(
    "/home/robot/Dopey-DD2419/src/object_detection/src/test_images/oakie.jpg",
    cv.IMREAD_COLOR,
)
lower = np.array([67, 71, 70])
upper = np.array([190, 190, 188])


mask = cv.inRange(img, lower, upper)

masked = cv.bitwise_and(img, img, mask=mask)

result = img - masked
plt.imshow(img)
plt.show()
plt.imshow(masked)
plt.show()
plt.imshow(mask)
plt.show()
plt.imshow(result)
plt.show()
