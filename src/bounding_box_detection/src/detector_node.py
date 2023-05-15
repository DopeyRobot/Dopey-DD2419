#!/usr/bin/env python3

from typing import List
from collections import Counter
import rospy
import torch
from torchvision.transforms import ToTensor, Normalize
from detector import Detector
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
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
from take_photos.srv import takePic, takePicRequest
from play_tunes.srv import playTune, playTuneRequest
from bounding_box_detection.srv import add2ShortTerm, add2ShortTermRequest, add2ShortTermResponse, instanceNames, instanceNamesRequest, instanceNamesResponse
from bounding_box_detection.msg import StringArray

os.environ["PYTORCH_CUDA_ALLOC_CONF"] = "max_split_size_mb:512"
torch.cuda.set_per_process_memory_fraction(0.6, 0)
torch.cuda.empty_cache()

from functools import wraps
import time


def timeit(func):
    @wraps(func)
    def timeit_wrapper(*args, **kwargs):
        start_time = time.perf_counter()
        result = func(*args, **kwargs)
        end_time = time.perf_counter()
        total_time = end_time - start_time
        print(f"Function {func.__name__}{args} {kwargs} Took {total_time:.4f} seconds")
        return result

    return timeit_wrapper


class BoundingBoxNode:
    def __init__(self) -> None:
        rospy.wait_for_service("/take_pic", 5)
        rospy.wait_for_service("/playTune", 5)
        rospy.wait_for_service("/add2shortterm", 10)

        self.f = 30
        self.max_rec_dist = 1 # max distance at whinch objects should still be considered by the memory
        self.min_rec_dist = 0.01
        self.rate = rospy.Rate(self.f)

        self.camera_topic = "/camera/color/image_raw"
        self.out_image_topic = "/camera/color/image_bbs"
        self.depth_topic = "/camera/aligned_depth_to_color/image_raw"
        self.camera_info_topic = "/camera/aligned_depth_to_color/camera_info"
        self.model_path = "/home/dopey/dd2419_ws/src/bounding_box_detection/src/models/det_2023-03-27_17-22-20-042133.pt"

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
        self.new_names = StringArray()
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

        self.short_term_mem_proxy = rospy.ServiceProxy("/add2shortterm", add2ShortTerm)
        self.new_names_sub = rospy.Subscriber("/new_names", StringArray, self.new_names_cb)
        self.pic_service = rospy.ServiceProxy("/take_pic", takePic)
        self.play_tune_service = rospy.ServiceProxy("/playTune", playTune)


        self.run()

    def new_names_cb(self, msg:StringArray):
        self.new_names = msg


    def take_pic(self, path):
        req = takePicRequest()
        req.path = String(path)
        self.pic_service(req)

    def play_tune(self, name):
        req = playTuneRequest()
        req.tuneToPlay = String(name)
        self.play_tune_service(req)

    def save_pic(self, image, path):
        cv2.imwrite(path, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))

    def image_callback(self, msg):
        timestamp = msg.header.stamp
        self.ros_img = msg
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        self.image = PILImage.fromarray(image)
        self.array_image = np.asarray(image)

        bbs = self.predict(self.image.copy())
        # supress multiple bbs
        self.bbs = utils.non_max_suppresion(
            bbs, confidence_threshold=0.80, diff_class_thresh=0.75
        )
        # add bbs to image and publish
        bb_image = self.show_bbs_in_image(self.bbs, self.array_image)

        for bb in self.bbs:
            class_name = String(self.get_class_name(bb, self.array_image))
            position = self.project_bb(bb)
            if np.linalg.norm(position) > self.max_rec_dist or np.linalg.norm(position)<self.min_rec_dist:
                continue
            position = self.convert_to_map(position)
            add_req = add2ShortTermRequest(class_name, position, rospy.Time.now())
            self.short_term_mem_proxy(add_req)

        # rospy.loginfo("length of the new names: " + str(len(self.new_names.array)))
        if len(self.new_names.array) > 0:
            path = "/home/dopey/dd2419_ws/src/bounding_box_detection/src/evidence"
            names = [s.data for s in self.new_names.array]
            instances = "_".join(names)
            full_path = path + "/" + instances + ".jpg"
            # self.save_pic(bb_image, full_path)
            for name in names:
                rospy.loginfo("NOW playing tune for name" + name )
                self.play_tune(name)

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

        bbs = self.model.decode_output(out, 0.8, scale_bb=False)[0]
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

        return image

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
        # image = cv2.resize(self.array_image, self.input_size)
        image = ToTensor()(image)
        image = Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])(image)
        if self.cuda:
            return image.to("cuda")

        return image

    def get_class_name(self, bb: utils.BoundingBox, image):
        """
        returns the class name of a bounding box
        """
        color = utils.detect_color(bb, image)

        return color

    def convert_to_map(self, position: np.ndarray):
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

        map_position = np.array(
            [
            transformed_pose.pose.position.x,
            transformed_pose.pose.position.y,
            transformed_pose.pose.position.z
            ]
        )

        return map_position


    def run(self):
        while not rospy.is_shutdown():

            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("bbdetector")
    BoundingBoxNode()
    rospy.spin()
