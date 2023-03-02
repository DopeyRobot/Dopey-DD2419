#!/usr/bin/env python3

import rospy
import torch
from detector import Detector
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from PIL import Image as PILImage
import numpy as np
import utils
import os
import time
os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "max_split_size_mb:512"
torch.cuda.set_per_process_memory_fraction(0.8, 0)
torch.cuda.empty_cache()
class BoundingBoxNode:
    def __init__(self) -> None:
        self.camera_topic = "/camera/color/image_raw"
        self.out_image_topic = "/camera/color/image_bbs"
        self.model_path = "/home/robot/dd2419_ws/src/bounding_box_detection/src/det_2023-03-02_14-23-34-390133.pt"
        self.image_subscriber = rospy.Subscriber(self.camera_topic, Image, self.image_callback)
        self.image_publisher = rospy.Publisher(self.out_image_topic, Image, queue_size=10)
        self.bridge = CvBridge()
        self.bb_publisher = None
        self.f = 60
        self.verbose = False
        self.rate = rospy.Rate(self.f)
        # self.model = Detector()
        self.model=torch.load(self.model_path)
        self.model.eval()
        self.cuda = torch.cuda.is_available()
        self.ros_img = None
        
        if self.cuda:
            self.model.cuda()

        self.image = None

        self.run()

    def image_callback(self, msg):
        self.ros_img = msg
        image = cv2.cvtColor(self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough'), cv2.COLOR_BGR2RGB)
        self.image = PILImage.fromarray(image)
        self.array_image = np.asarray(image)
    def run(self):
        while not rospy.is_shutdown():
            if self.image is not None:
                if self.verbose:
                    start = time.time()
                torch_image, _ = self.model.input_transform(self.image.copy(), [], validation=True)
                if self.cuda:
                    torch_image = torch_image.to("cuda")
                out = self.model(torch_image.unsqueeze(0)).cpu()
                bbs = self.model.decode_output(out, 0.5, scale_bb=True)[0]
                if len(bbs) >0:
                    bbs = utils.non_max_suppresion(bbs, confidence_threshold=0.80, diff_class_thresh=0.75)
                    image = utils.draw_bb_on_image(self.array_image, bbs, utils.CLASS_DICT)
                    ros_img = self.bridge.cv2_to_imgmsg(image,  encoding="passthrough")
                    self.image_publisher.publish(ros_img)
                else:
                    self.image_publisher.publish(self.ros_img)
                
                if self.verbose:
                    rospy.loginfo(f"full inference time = {time.time() - start}")
            
            self.rate.sleep()
        

if __name__ == "__main__":
    rospy.init_node("bbdetector")
    BoundingBoxNode()
    rospy.spin()