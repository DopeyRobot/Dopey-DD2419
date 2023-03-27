#!/usr/bin/env python3

from typing import List
from collections import Counter
import rospy
import torch
from torchvision.transforms import ToTensor, Normalize
from detector import Detector
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import cv2
from PIL import Image as PILImage
from memory import ShortTermMemory, LongTermMemory
import numpy as np
import utils
import os
import time
import matplotlib.pyplot as plt
from tf2_geometry_msgs import PoseStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, TransformStamped

os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "max_split_size_mb:512"
torch.cuda.set_per_process_memory_fraction(0.6, 0)
torch.cuda.empty_cache()


class BoundingBoxNode:
    def __init__(self) -> None:
        self.f = 60
        self.rate = rospy.Rate(self.f)

        self.camera_topic = "/camera/color/image_raw"
        self.out_image_topic = "/camera/color/image_bbs"
        self.depth_topic = "/camera/aligned_depth_to_color/image_raw"
        self.camera_info_topic = "/camera/aligned_depth_to_color/camera_info"
        self.model_path = "/home/robot/Dopey-DD2419/src/bounding_box_detection/src/det_2023-03-22_13-57-06-487490.pt"

        self.camera_frame = "camera_color_optical_frame"
        self.map_frame = "map"
        self.input_size = (640, 480)
        self.model = torch.load(self.model_path)
        self.model.eval()
        self.cuda = torch.cuda.is_available()

        if self.cuda:
            self.model.cuda()
            self.model_trace = torch.jit.trace(
                self.model, torch.rand((1, 3, 480, 640)).to("cuda")
            )
        self.broadcaster = TransformBroadcaster()
        self.buffer = Buffer(rospy.Duration(1200.0))
        self.listener = TransformListener(self.buffer)

        self.bbs: List[utils.BoundingBox] = []

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
        self.verbose = False
        # self.model = Detector()

        self.camera_info = rospy.wait_for_message(
            self.camera_info_topic, CameraInfo, rospy.Duration(5)
        )
        self.K = np.array(self.camera_info.K).reshape(3, 3)

        self.short_term_memory = ShortTermMemory()
        self.long_term_memory = LongTermMemory(frames_needed_for_reconition=15)

        self.run()

    def image_callback(self, msg):
        start = time.time()
        timestamp = msg.header.stamp
        self.ros_img = msg
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.image = PILImage.fromarray(image)
        self.array_image = np.asarray(image)

        bbs = self.predict(self.image.copy())
        # supress multiple bbs
        self.bbs = utils.non_max_suppresion(
            bbs, confidence_threshold=0.70, diff_class_thresh=0.75
        )
        # add bbs to image and publish
        self.show_bbs_in_image(self.bbs, self.array_image)

        for bb in self.bbs:
            class_name = self.get_class_name(bb)
            position = self.project_bb(bb)
            self.short_term_memory.add(class_name, position, timestamp)

        self.long_term_memory.checkForObjectsToRemember(
            timestamp, self.short_term_memory
        )
        self.publish_long_term_memory()
        t = time.time() - start
        print(f"inference time = {t}, FPS = {1/t}")

    def depth_callback(self, msg):
        self.ros_depth = msg
        depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.depth = PILImage.fromarray(depth)
        self.array_depth = np.asarray(depth)

    def predict(self, image) -> List[utils.BoundingBox]:
        """
        returns a raw list of bounding boxes
        """
        torch_image = self.image_transforms(image)

        out = self.model_trace(torch_image.unsqueeze(0)).cpu()

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

    def project_bb(self, bb: utils.BoundingBox):
        """
        returns the projected 3d postion of the center of a bounding box in world coordinates
        in camera frame
        """
        if self.array_depth is None or self.K is None:
            return np.array([0, 0, 0])
        center_x = int(bb["x"] + bb["width"] / 2)
        center_y = int(bb["y"] + bb["height"] / 2)
        world_z = self.array_depth[center_y, center_x]
        fx = self.K[0, 0]
        fy = self.K[1, 1]
        world_x = (center_x - self.K[0, 2]) * world_z / fx
        world_y = (center_y - self.K[1, 2]) * world_z / fy

        return np.array([world_x / 1000, world_y / 1000, world_z / 1000])

    def get_frame_name(self, class_name: str, instance_id: str):
        """
        returns the full name of a frame
        """
        return class_name + "_" + instance_id

    def image_transforms(self, image):
        image = cv2.resize(self.array_image, self.input_size)
        image = ToTensor()(image)
        image = Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])(image)
        if self.cuda:
            return image.to("cuda")

        return image

    def get_class_name(self, bb: utils.BoundingBox):
        """
        returns the class name of a bounding box
        """
        class_name = utils.CLASS_DICT[bb["category_id"]]

        return class_name

    def publish_to_tf(self, frame_name: str, position: np.ndarray):
        """
        Publish a transform from the camera frame to the map frame
        """
        pose = PoseStamped()
        pose.header.frame_id = self.camera_frame
        pose.header.stamp = self.ros_img.header.stamp
        pose.pose.position.x = position[0]
        pose.pose.position.y = position[1]
        pose.pose.position.z = position[2]

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

        t.child_frame_id = frame_name
        self.broadcaster.sendTransform(t)

    def publish_long_term_memory(self):
        for instance in self.long_term_memory:
            self.publish_to_tf(
                self.get_frame_name(instance.instance_name, ""),
                instance.position,
            )

    def run(self):
        while not rospy.is_shutdown():

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("bbdetector")
    BoundingBoxNode()
    rospy.spin()
