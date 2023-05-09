import cv2 as cv
import numpy as np

final_img = cv.imread("C:/KTH/Robotics Project course/Dopey-DD2419/src/object_detection/src/test_images/14.png")
thresholded = cv.cvtColor(final_img, cv.COLOR_BGR2GRAY)
_,thresholded = cv.threshold(thresholded, 122, 255, cv.THRESH_BINARY)
print(thresholded.shape)
final_img = thresholded
final_img = cv.bitwise_not(final_img)

(numLabels, labels, stats, centroids) = cv.connectedComponentsWithStats(final_img.astype(np.uint8))
print(numLabels)
print(labels)
print(stats)
print(centroids)

# loop over the number of unique connected component labels
for i in range(0, numLabels):
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
    output = final_img
    cv.rectangle(output, (x, y), (x + w, y + h), (140, 140, 140), 3)
    cv.circle(output, (int(cX), int(cY)), 4, (140, 140, 140), -1)

    # connected component ID
    componentMask = (labels == i).astype("uint8") * 255
    # show our output image and connected component mask
    cv.imshow("Output", output)
    cv.imshow("Connected Component", componentMask)
cv.waitKey(0)






cv.circle(output, (100, 200), 4, (0, 0, 255), -1)

# connected component ID
componentMask = (labels == i).astype("uint8") * 255
# show our output image and connected component mask
cv.imshow("Output", output)
cv.imshow("Connected Component", componentMask)
cv.waitKey(0)