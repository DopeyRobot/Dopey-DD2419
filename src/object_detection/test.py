#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

img = cv.imread(
    "C:/KTH/Robotics Project course/Dopey-DD2419/src/object_detection/src/test_images/cubes.jpg",
    cv.IMREAD_COLOR,
)
# Color Filter
lower = np.array([67, 71, 70])
upper = np.array([190, 190, 188])
mask = cv.inRange(img, lower, upper)
masked = cv.bitwise_and(img, img, mask=mask)
result = img - masked
plt.imshow(img)
plt.title("Original")
plt.show()
plt.imshow(mask)
plt.title("Mask")
plt.show()
plt.imshow(masked)
plt.title("Masked")
plt.show()
plt.imshow(result)
plt.title("Result = Original - Masked")
plt.show()

#Thresholding
_,thresholded = cv.threshold(masked, 80, 255, cv.THRESH_BINARY)
thresholded = cv.cvtColor(thresholded, cv.COLOR_BGR2GRAY)
_,thresholded = cv.threshold(thresholded, 122, 255, cv.THRESH_BINARY)

print(np.unique(thresholded))
print(thresholded.shape)
plt.imshow(thresholded)
plt.title("Thresholded")
plt.show()

#Morphological operations
closed = cv.morphologyEx(thresholded, cv.MORPH_CLOSE, np.ones((3,3), np.uint8), iterations=2)
plt.imshow(closed)
plt.title("Closed")
plt.show()
open = cv.morphologyEx(closed, cv.MORPH_OPEN, np.ones((5,5), np.uint8), iterations=5)

#Cut the bottom of the image and make it white
cut_img = open
cut_img = cut_img[:-100, :]
plt.imshow(cut_img)
plt.title("cut_img")
plt.show()

white = np.ones(img.shape[:2], dtype=np.uint8)*255
white[:-100,:] = cut_img
final_img = white
plt.imshow(final_img)
plt.title("Final")
plt.show()

# Find the coords of the center of the biggest blob
final_img = cv.bitwise_not(final_img) #invert image because connectedComponentsWithStats assumes white as components
(numLabels, labels, stats, centroids) = cv.connectedComponentsWithStats(final_img.astype(np.uint8))
print(numLabels)
print(labels)
print(stats)
print(centroids)
max_blob_area = -1
max_blob = -1
(xFinal,yFinal) = (-1,-1)
# loop over the number of unique connected component labels
for i in range(1, numLabels):
    # if this is the first component then we examine the
    # *background* (typically we would just ignore this
    # component in our loop)
    if i == 0:
        text = "examining component {}/{} (background)".format(
        i + 1, numLabels)
    # otherwise, we are examining an actual connected component
    else:
        text = "examining component {}/{}".format( i + 1, numLabels)
    # print a status message update for the current connected
    # component
    print("[INFO] {}".format(text))
    # extract the connected component statistics and centroid for
    # the current label
    x = stats[i, cv.CC_STAT_LEFT]
    y = stats[i, cv.CC_STAT_TOP]
    w = stats[i, cv.CC_STAT_WIDTH]
    h = stats[i, cv.CC_STAT_HEIGHT]
    area = stats[i, cv.CC_STAT_AREA]
    (cX, cY) = centroids[i]
    if max_blob_area > area:
        max_blob_area = area
        max_blob = i
        (xFinal,yFinal) = (cX,cY)
    output = final_img
    cv.rectangle(output, (x, y), (x + w, y + h), (140, 140, 140), 3)
    cv.circle(output, (int(cX), int(cY)), 4, (140, 140, 140), -1)
    # connected component ID
    componentMask = (labels == i).astype("uint8") * 255
    # show our output image and connected component mask

    cv.imshow("Output", output)
    cv.imshow("Connected Component", componentMask)
cv.waitKey(0)

