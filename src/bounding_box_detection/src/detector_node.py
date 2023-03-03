#!/usr/bin/env python3

from typing import List
import rospy
import torch
from detector import Detector
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from PIL import Image as PILImage
import numpy as np
import utils
import os
import time
import matplotlib.pyplot as plt
from tf2_geometry_msgs import PoseStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, TransformStamped

os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "max_split_size_mb:512"
torch.cuda.set_per_process_memory_fraction(0.8, 0)
torch.cuda.empty_cache()


class BoundingBoxNode:
    def __init__(self) -> None:
        self.f = 60
        self.rate = rospy.Rate(self.f)

        self.camera_topic = "/camera/color/image_raw"
        self.out_image_topic = "/camera/color/image_bbs"
        self.depth_topic = "/camera/aligned_depth_to_color/image_raw"
        self.camera_info_topic = "/camera/aligned_depth_to_color/camera_info"
        self.model_path = "/home/robot/dd2419_ws/src/bounding_box_detection/src/det_2023-03-02_14-23-34-390133.pt"

        self.camera_frame = "camera_color_optical_frame"
        self.map_frame = "map"

        self.broadcaster = TransformBroadcaster()
        self.buffer = Buffer(rospy.Duration(1200.0))
        self.listener = TransformListener(self.buffer)

        self.image_subscriber = rospy.Subscriber(
            self.camera_topic, Image, self.image_callback
        )
        self.depth_subscriber = rospy.Subscriber(
            self.depth_topic, Image, self.depth_callback
        )
        self.image_publisher = rospy.Publisher(
            self.out_image_topic, Image, queue_size=10
        )
        self.bridge = CvBridge()

        self.ros_img = None
        self.ros_depth = None
        self.image = None
        self.array_image = None
        self.depth = None
        self.array_depth = None
        self.camera_info = rospy.wait_for_message(
            self.camera_info_topic, CameraInfo, rospy.Duration(5)
        )
        self.K = np.array(self.camera_info.K).reshape(3, 3)

        self.verbose = False
        # self.model = Detector()
        self.model = torch.load(self.model_path)
        self.model.eval()
        self.cuda = torch.cuda.is_available()

        if self.cuda:
            self.model.cuda()

        self.run()

    def image_callback(self, msg):
        self.ros_img = msg
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.image = PILImage.fromarray(image)
        self.array_image = np.asarray(image)

    def depth_callback(self, msg):
        self.ros_depth = msg
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.depth = PILImage.fromarray(depth)
        self.array_depth = np.asarray(depth)

    def predict(self, image) -> List[utils.BoundingBox]:
        torch_image, _ = self.model.input_transform(image, [], validation=True)
        if self.cuda:
            torch_image = torch_image.to("cuda")
        out = self.model(torch_image.unsqueeze(0)).cpu()
        bbs = self.model.decode_output(out, 0.5, scale_bb=True)[0]
        return bbs

    def show_bbs_in_image(self, bbs: List[utils.BoundingBox], image) -> None:
        if len(bbs) > 0:
            image = utils.draw_bb_on_image(image, bbs, utils.CLASS_DICT)
            ros_img = self.bridge.cv2_to_imgmsg(
                cv2.cvtColor(image, cv2.COLOR_RGB2BGR), encoding="passthrough"
            )
            self.image_publisher.publish(ros_img)
        else:
            self.image_publisher.publish(self.ros_img)

    def project_bb(self, bbs: List[utils.BoundingBox]):

        for bb in bbs:
            center_x = int(bb["x"] + bb["width"] / 2)
            center_y = int(bb["y"] + bb["height"] / 2)
            world_z = self.array_depth[center_y, center_x]
            fx = self.K[0, 0]
            fy = self.K[1, 1]
            world_x = (center_x - self.K[0, 2]) * world_z / fx
            world_y = (center_y - self.K[1, 2]) * world_z / fy

            pose = PoseStamped()
            pose.header.frame_id = self.camera_frame
            pose.header.stamp = self.ros_img.header.stamp
            pose.pose.position.x = world_x / 1000
            pose.pose.position.y = world_y / 1000
            pose.pose.position.z = world_z / 1000

            pose.pose.orientation.x = 0.5
            pose.pose.orientation.y = 0.5
            pose.pose.orientation.z = 0.5
            pose.pose.orientation.w = 0.5

            transformed_pose = self.buffer.transform(
                pose, self.map_frame, rospy.Duration(1.0)
            )

            t = TransformStamped()
            t.header.stamp = transformed_pose.header.stamp
            t.header.frame_id = "map"

            t.transform.translation.x = transformed_pose.pose.position.x
            t.transform.translation.y = transformed_pose.pose.position.y
            t.transform.translation.z = transformed_pose.pose.position.z

            t.transform.rotation.x = transformed_pose.pose.orientation.x
            t.transform.rotation.y = transformed_pose.pose.orientation.y
            t.transform.rotation.z = transformed_pose.pose.orientation.z
            t.transform.rotation.w = transformed_pose.pose.orientation.w

            t.child_frame_id = utils.CLASS_DICT[bb["category_id"]]
            self.broadcaster.sendTransform(t)

    def run(self):
        while not rospy.is_shutdown():
            if self.image is not None:
                if self.verbose:
                    start = time.time()
                bbs = self.predict(self.image.copy())
                # supress multiple bbs
                bbs = utils.non_max_suppresion(
                    bbs, confidence_threshold=0.80, diff_class_thresh=0.75
                )
                # add bbs to image and publish
                self.show_bbs_in_image(bbs, self.array_image)

                if self.verbose:
                    rospy.loginfo(f"full inference time = {time.time() - start}")

            if self.depth is not None and self.K is not None and bbs is not None:
                self.project_bb(bbs)

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("bbdetector")
    BoundingBoxNode()
    rospy.spin()
