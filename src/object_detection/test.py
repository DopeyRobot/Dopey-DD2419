#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt

img = cv.imread(
    #"/home/dopey/dd2419_ws/src/object_detection/src/test_images/blue_cube.jpg", # Robot Computer
    "C:/KTH/Robotics Project course/Dopey-DD2419/src/object_detection/src/test_images/slush.jpg", # RC Computer
    cv.IMREAD_COLOR,
)
plt.imshow(img)
plt.title("Original")
plt.show()
# Color Filter
############BACKGROUND COLORS FILTER############
# lower = np.array([67, 71, 70])
# upper = np.array([190, 190, 188])
lower = np.array([75, 74, 76])
upper = np.array([185, 185, 185])

mask = cv.inRange(img, lower, upper)
masked = cv.bitwise_and(img, img, mask=mask)
result = img - masked

plt.imshow(mask)
plt.title("Mask")
plt.show()
plt.imshow(masked)
plt.title("Masked")
plt.show()
# plt.imshow(result)
# plt.title("Result = Original - Masked")
# plt.show()

#Thresholding
_,thresholded = cv.threshold(masked, 80, 255, cv.THRESH_BINARY)
thresholded = cv.cvtColor(thresholded, cv.COLOR_BGR2GRAY)
_,thresholded = cv.threshold(thresholded, 122, 255, cv.THRESH_BINARY)

print(np.unique(thresholded))
print(thresholded.shape)
plt.imshow(thresholded)
plt.title("Thresholded")
plt.show()
# Use fillPoly() function and give input as
# image, end points, color of polygon
# Here color of polygon will be white
cut_img = thresholded.copy()
points = np.array([[0, 479], [0, 330], [450, 410], [540,479]])
cv.fillPoly(cut_img, pts=[points], color=(
    255
))
plt.imshow(cut_img)
plt.title("cut_img")
plt.show()

#!!!!!!!!!!!!!!!!!!!!!!!!!! NOTE: MAYBE BETTER TO OPEN FIRST CLOSE SECOND !!!!!!! CHECK IT!!!!!!!!!!!!!
close_first_open_second=False
if close_first_open_second:
    cut_img = cv.bitwise_not(cut_img)

#Morphological operations
closed = cv.morphologyEx(cut_img, cv.MORPH_CLOSE, np.ones((3,3), np.uint8), iterations=3)
plt.imshow(closed)
plt.title("Closed")
plt.show()
open = cv.morphologyEx(closed, cv.MORPH_OPEN, np.ones((4,4), np.uint8), iterations=3)
plt.imshow(open)
plt.title("Closed and Opened")
plt.show()

#Cut the bottom of the image and make it white
final_img = open
if not close_first_open_second:
    final_img = cv.bitwise_not(final_img) #invert image because connectedComponentsWithStats assumes white as components


##############################################
######Technique #1 FIND THE BIGGEST BLOB######
##############################################

(numLabels, labels, stats, centroids) = cv.connectedComponentsWithStats(final_img.astype(np.uint8))
print(numLabels)
print(labels)
print(stats)
print(centroids)
max_blob_area = -1
max_blob = -1
(CxFinal,CyFinal,xFinal,yFinal,hFinal,wFinal) = (-1,-1,-1,-1,-1,-1)
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
    if max_blob_area < area:
        max_blob_area = area
        max_blob = i
        (CxFinal,CyFinal,xFinal,yFinal,hFinal,wFinal) = (cX,cY,x,y,h,w)
    output = final_img.copy()
    cv.rectangle(output, (x, y), (x + w, y + h), (140, 140, 140), 3)
    cv.circle(output, (int(cX), int(cY)), 4, (140, 140, 140), -1)
    # connected component ID
    componentMask = (labels == i).astype("uint8") * 255
    # show our output image and connected component mask

    # plt.imshow(output)
    # plt.title("output")
    # plt.show()
    # plt.imshow(componentMask)
    # plt.title("components")
    # plt.show()

print("center for max blob = " + str((CxFinal,CyFinal)) + " with area = " + str(max_blob_area))
max_blob_img = (labels == max_blob).astype("uint8") * 255
plt.imshow(max_blob_img)
plt.title("max_blob_img")
plt.show()


##############################################
# FIND THE LARGEST CONTOUR AND FIT AN ELLIPSE#
##############################################
# plt.imshow(final_img)
# plt.title("final_img")
# plt.show()
canny_output = cv.Canny(final_img, 100, 200)
# plt.imshow(canny_output)
# plt.title("canny_output")
# plt.show()
    
contours, _ = cv.findContours(canny_output, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
max_contour_area = -1
max_contour = -1
(cX_final,cY_final,short_axis_final,long_axis_final,angle_final) = (-1,-1,-1,-1,-1)
    # Find the rotated rectangles and ellipses for each contour
minEllipse = [None]*len(contours)
print("countours found: "+str(len(contours)))
minEllipse = [None]*len(contours)
for i, c in enumerate(contours):
    if c.shape[0] > 5:
        minEllipse[i] = cv.fitEllipse(c)
        center = minEllipse[i][0]
        cX = center[0]
        cY = center[1]
        size = minEllipse[i][1]
        short_axis = min(size)
        long_axis = max(size)
        area = np.pi*short_axis*long_axis

        print("area = "+str(area))
        if area > max_contour_area:
                max_contour_area = area
                max_contour = i
                angle = minEllipse[i][2]
                (cX_final,cY_final,short_axis_final,long_axis_final,angle_final) = (cX,cY,long_axis,short_axis,angle)
# Draw contours + rotated rects + ellipses

drawing = np.zeros((canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)
ellipse_color = (255,0,255)

for i, c in enumerate(contours):
    # contour
    cv.drawContours(drawing, contours, i, ellipse_color)
    # ellipse
    if c.shape[0] > 5:
        cv.ellipse(drawing, minEllipse[i], ellipse_color, 2)
        cv.circle(drawing, (int(cX), int(cY)), 4, ellipse_color, -1)


max_contour_img = np.zeros((canny_output.shape[0], canny_output.shape[1], 3), dtype=np.uint8)
cv.drawContours(max_contour_img, contours, max_contour, ellipse_color)
cv.ellipse(max_contour_img, minEllipse[max_contour], ellipse_color, 2)
cv.circle(max_contour_img, (int(cX_final), int(cY_final)), 4, ellipse_color, -1)
plt.imshow(max_contour_img)
plt.title("max_contour_img")
plt.show()

