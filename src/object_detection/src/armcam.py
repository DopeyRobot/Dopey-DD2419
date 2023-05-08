#!/usr/bin/env python3
# Standard imports
import cv2
import numpy as np;
 
# Read image
im = cv2.imread("/home/robot/dopey_ws/src/object_detection/src/blob.jpg", cv2.IMREAD_GRAYSCALE)
 
# # Set up the detector with default parameters.
# detector = cv2.SimpleBlobDetector()
 
# # Detect blobs.
# keypoints = detector.detect(im)

params = cv2.SimpleBlobDetector_Params()

ver = (cv2.__version__).split('.')
if int(ver[0]) < 3 :
    detector = cv2.SimpleBlobDetector(params)
else : 
    detector = cv2.SimpleBlobDetector_create(params)

detector.empty() # <- now works
keypoints = detector.detect(im) # <- now works
 
# Draw detected blobs as red circles.
# cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
im_with_keypoints = cv2.drawKeypoints(im, keypoints, np.array([]), (0,0,255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
 
# Show keypoints
cv2.imshow("Keypoints", im_with_keypoints)
cv2.waitKey(0)

print(keypoints)
# for keypt in keypoints:
#     print(keypt.pt)
#     print(keypt.size)
#     print(keypt.angle)
#     print(keypt.response)
#     print(keypt.octave)
#     print(keypt.class_id)
#     print("\n")