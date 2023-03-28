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
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse

# Instantiate CvBridge


class TakePhotos:
    def __init__(self):
        # Define your image topic
        image_topic = (
            "/camera/color/image_raw"  # "/usb_cam/image_raw" for the arm camera
        )
        # Set up your subscriber and define its callback
        self.bridge = CvBridge()

        rospy.Subscriber(image_topic, Image, self.image_callback)
        self.take_pic_service = rospy.Service(
            "take_pic", String, self.take_pic_callback
        )
        # Spin until ctrl + c
        self.image = None
        self.counter = 0
        self.f = 10
        self.rate = rospy.Rate(self.f)

        self.run()

    def image_callback(self, msg):
        try:
            # Convert your ROS Image message to OpenCV2
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except:
            print("err")

    def run(self):
        while not rospy.is_shutdown():
            try:
                input()
                cv2.imwrite(
                    f'./src/take_photos/photos/dp_img{datetime.now().strftime("%Y_%m_%d_%H_%M_%S_%f%")}.jpeg',
                    self.image,
                )
                print("pic taken")
            except Exception as e:
                print(e)
                print(self.image)
                print("no pic")
            self.rate.sleep()

    def take_pic_callback(self, req):
        path = req.data
        try:
            # Convert your ROS Image message to OpenCV2
            cv2.imwrite(
                path,
                self.image,
            )
            print("pic taken")
        except Exception as e:
            print("no pic")
        return EmptyResponse()


if __name__ == "__main__":
    rospy.init_node("image_listener")
    TakePhotos()
    rospy.spin()
