#!/usr/bin/env python3
# Standard imports
import cv2
import numpy as np
import matplotlib.pyplot as plt

# Read image
img = cv2.imread(
    "/home/robot/Dopey-DD2419/src/object_detection/src/test_images/ball_r.jpg",
    cv2.IMREAD_COLOR,
)[:-100, :]


# blurred_img = cv2.GaussianBlur(img, (15, 15), 0)
# saliency = cv2.saliency.StaticSaliencySpectralResidual_create()
# success, saliencyMap = saliency.computeSaliency(blurred_img)
# # threshMap = cv2.threshold(
# #     (saliencyMap * 255).astype("uint8"), 125, 255, cv2.THRESH_BINARY
# # )[1]

# threshMap = cv2.threshold(
#     (blurred_img * 255).astype("uint8"), 150, 255, cv2.THRESH_BINARY
# )[1]
# plt.imshow(img, cmap="gray")
# plt.show()
# plt.imshow(saliencyMap, cmap="gray")
# plt.show()
# plt.imshow(threshMap, cmap="gray")
# plt.show()

# define range of blue color in HSV
lower_gray = np.array([93, 95, 71])
upper_gray = np.array([130, 255, 255])
# Threshold the HSV image to get only blue colors

cap = img
while 1:
    # Take each frame
    frame = img
    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # define range of blue color in HSV
    lower_blue = np.array([110, 50, 50])
    upper_blue = np.array([130, 255, 255])
    # Threshold the HSV image to get only blue colors
    mask = cv2.inRange(hsv, lower_blue, upper_blue)
    # Bitwise-AND mask and original image
    res = cv2.bitwise_and(frame, frame, mask=mask)
    cv2.imshow("frame", frame)
    cv2.imshow("mask", mask)
    cv2.imshow("res", res)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break
cv2.destroyAllWindows()


# plt.imshow(im, cmap="gray")
# plt.title("im")
# plt.show()
# t1 = 150
# bs = 5
# c = 15
# thresholded = cv2.adaptiveThreshold(
#     im, t1, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, bs, c
# )
# plt.imshow(thresholded, cmap="gray")
# plt.title("Thresholded")
# plt.show()

# closed = cv2.morphologyEx(thresholded, cv2.MORPH_OPEN, np.ones((3, 3)))
# plt.imshow(closed, cmap="gray")
# plt.title("Closed")
# plt.show()
# inflated = cv2.dilate(
#     closed, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3)), iterations=1
# )

# plt.imshow(inflated, cmap="gray")
# plt.title("Inflated")
# plt.show()

# dilated = cv2.erode(
#     inflated, cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5)), iterations=2
# )
# plt.imshow(dilated, cmap="gray")
# plt.title("Dilated")
# plt.show()

# blurred = cv2.GaussianBlur(dilated, (51, 51), 0)
# plt.imshow(blurred, cmap="gray")
# plt.title("Blurred")
# plt.show()

# _, thresholded_again = cv2.threshold(blurred, 80, 255, cv2.THRESH_BINARY)
# plt.imshow(thresholded_again, cmap="gray")
# plt.title("Thresholded again")
# plt.show()
# # Set up the detector with default parameters.
# detector = cv2.SimpleBlobDetector()

# # Detect blobs.
# keypoints = detector.detect(im)

# params = cv2.SimpleBlobDetector_Params()
# detector = cv2.SimpleBlobDetector_create(params)

# detector.empty()  # <- now works
# keypoints = detector.detect(thresholded_again)  # <- now works

# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
# im_with_keypoints = cv2.drawKeypoints(
#     im, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS
# )

# Show keypoints
# plt.imshow(cv2.cvtColor(im_with_keypoints, cv2.COLOR_BGR2RGB))
# plt.show()

# print(keypoints)
# for keypt in keypoints:
#     print(keypt.pt)
#     print(keypt.size)
#     print(keypt.angle)
#     print(keypt.response)
#     print(keypt.octave)
#     print(keypt.class_id)
#     print("\n")
