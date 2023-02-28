#!/usr/bin/env python3

import rospy
import torch
from detector import Detector

class BoundingBoxNode:
    def __init__(self) -> None:
        self.camera_topic = ""
        self.model_path = "src/bounding_box_detection/src/det_2023-02-21_19-29-24-367975.pt"
        self.image_subscriber = None
        self.bb_publisher = None
        self.f = 2
        self.rate = rospy.Rate(self.f)
        self.model = Detector()
        self.model.load_state_dict(torch.load(self.model_path))
        self.model.eval()

        self.run()

    def image_callback(self, msg):
        pass

    def run(self):
        while not rospy.is_shutdown():
            rospy.loginfo("working")
            self.rate.sleep()
        

if __name__ == "__main__":
    rospy.init_node("bbdetector")
    BoundingBoxNode()
    rospy.spin()