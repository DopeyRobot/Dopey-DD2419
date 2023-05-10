#! /usr/bin/python3
# Copyright (c) 2015, Rethink Robotics, Inc.

# Using this CvBridge Tutorial for converting
# ROS images to OpenCV2 images
# http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython

# Using this OpenCV2 tutorial for saving Images:
# http://opencv-python-tutroals.readthedocs.org/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html

# rospy for the subscriber
from datetime import datetime

# OpenCV2 for saving an image
import cv2
import rospy

# ROS Image message -> OpenCV2 image converter
from cv_bridge import CvBridge, CvBridgeError

# ROS Image message
from sensor_msgs.msg import Image
from take_photos.srv import takePic, takePicRequest, takePicResponse
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

# Instantiate CvBridge


class TakePhotos:
    def __init__(self):
        # Define your image topic
        self.image_topic = (
            "/camera/color/image_raw"  # "/usb_cam/image_raw" for the arm camera
        )
        self.image_topic_arm = (
            "/usb_cam/image_rect_color"  # "/usb_cam/image_raw" for the arm camera
        )
        # Set up your subscriber and define its callback
        self.bridge = CvBridge()

        self.take_pic_service = rospy.Service(
            "take_pic", takePic, self.take_pic_callback
        )
        self.take_pic_service_arm = rospy.Service(
            "take_pic_arm", takePic, self.take_pic_arm_callback
        )
        # Spin until ctrl + c
       
    def take_pic_callback(self, req:takePicRequest):
        path = req.path.data
        try:
            msg = rospy.wait_for_message(self.image_topic, Image, timeout=2)
            # Convert your ROS Image message to OpenCV2
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Convert your ROS Image message to OpenCV2
            cv2.imwrite(
                path,
                image,
            )
            print("pic taken")
        except Exception as e:
            print("no pic")
        return EmptyResponse()
    
    def take_pic_arm_callback(self, req:takePicRequest):
        path = req.path.data
        try:
            msg = rospy.wait_for_message(self.image_topic_arm, Image, timeout=2)
            # Convert your ROS Image message to OpenCV2
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Convert your ROS Image message to OpenCV2
            cv2.imwrite(
                path,
                image,
            )
            print("pic taken")
        except Exception as e:
            print("no pic")
        return EmptyResponse()


if __name__ == "__main__":
    rospy.init_node("photos_node")
    TakePhotos()
    rospy.spin()
