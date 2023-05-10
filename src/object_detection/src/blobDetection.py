#!/usr/bin/env python3
import cv2 as cv
import numpy as np
import matplotlib.pyplot as plt
import rospy
from typing import List, Tuple
from object_detection.srv import XnY, XnYResponse, XnYRequest
from bounding_box_detection.srv import setLocation, setLocationRequest
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CameraInfo
from tf2_geometry_msgs import PoseStamped
from std_msgs.msg import String, Bool
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, TransformStamped

class armcamDetection:
    def __init__(self) -> None:
        self.blob_service: rospy.Service = rospy.Service("/blob_detection", XnY, self.blob_detection_cb)
        self.arm_cam_topic = "/usb_cam/image_rect_color"
        self.arm_cam_info_topic = "/usb_cam/camera_info"
        self.arm_cam_frame = "usb_cam_frame"
        self.pickup_goal_topic = "/pickup_goal"
        self.current_obj = "/current_obj_id"
        self.image_topic = "/usb_cam/image_bbs"
        self.pickup_goal_pub = rospy.Publisher(self.pickup_goal_topic, PoseStamped, queue_size=10)
        self.bridge = CvBridge()
        self.broadcaster = TransformBroadcaster()
        self.buffer = Buffer(rospy.Duration(1200.0))
        self.listener = TransformListener(self.buffer)
        self.set_obj_location_caller = rospy.ServiceProxy("/set_obj_location", setLocation)
        self.cam_image_publisher = rospy.Publisher(
            self.image_topic, Image, queue_size=10
        )

    def project_blob(self, x, y) -> np.ndarray:
        """
        returns the projected 3d postion of the center of a bounding box in world coordinates
        in arm camera frame
        """
        camera_info = rospy.wait_for_message(self.arm_cam_info_topic, CameraInfo)
        K = np.array(camera_info.K).reshape(3, 3)
        world_z = 210
        fx = K[0, 0]
        fy = K[1, 1]
        world_x = (x - K[0, 2]) * world_z / fx
        world_y = (y - K[1, 2]) * world_z / fy

        return np.array([-world_y / 1000, world_x / 1000, world_z / 1000])

    def get_image(self) -> np.ndarray:
        msg = rospy.wait_for_message(self.arm_cam_topic, Image)
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        return image
    def draw_bounding_box(self, image,cx,cy, x, y, h, w) -> Image:
        image2 = image.copy()
        cv.rectangle(image2, (x, y), (x + w, y + h), (0, 255, 140), 3)
        cv.circle(image2, (int(cx), int(cy)), 4, (0, 255, 140), -1)
        print("publishing image")
        return self.bridge.cv2_to_imgmsg(image2, encoding="passthrough")
    
    def blob_detection_cb(self, req: XnYRequest) -> XnYResponse:
        image = self.get_image()
        (cx,cy,x,y,h,w) = self.detect_biggest_blob(image)
        
        # project the blob to 3d (camera frame)
        xyz_proj =self.project_blob(cx,cy)
        # convert to base_link frame
        pickUpPose = self.convert_to_base_link(xyz_proj)
        # publish as pickup goal
        self.publish_pickup_goal(pickUpPose)
        # remove object from map
        # instance_name = rospy.wait_for_message(self.current_obj, String)
        # print(instance_name)
        # self.set_obj_location_caller(String(instance_name), String("gripper"))
        # publish image with bounding box
        self.cam_image_publisher.publish(self.draw_bounding_box(image,cx,cy, x, y, h, w))

        if x == -1 or y == -1:
            return XnYResponse(-1,-1,-1,-1,-1,-1, Bool(False)) #TODO: check Types!! 
        return XnYResponse(cx,cy,x,y,h,w, Bool(True))
    
    def detect_biggest_blob(self, img) -> Tuple[int, int, int, int]:

        lower = np.array([67, 71, 70])
        upper = np.array([190, 190, 188])
        mask = cv.inRange(img, lower, upper)
        masked = cv.bitwise_and(img, img, mask=mask)
        #result = img - masked
        # plt.imshow(img)
        # plt.title("Original")
        # plt.show()
        # plt.imshow(mask)
        # plt.title("Mask")
        # plt.show()
        # plt.imshow(masked)
        # plt.title("Masked")
        # plt.show()
        # plt.imshow(result)
        # plt.title("Result = Original - Masked")
        # plt.show()

        #Thresholding
        _,thresholded = cv.threshold(masked, 80, 255, cv.THRESH_BINARY)
        thresholded = cv.cvtColor(thresholded, cv.COLOR_BGR2GRAY)
        _,thresholded = cv.threshold(thresholded, 122, 255, cv.THRESH_BINARY)

        # print(np.unique(thresholded))
        # print(thresholded.shape)
        # plt.imshow(thresholded)
        # plt.title("Thresholded")
        # plt.show()

        #Morphological operations
        closed = cv.morphologyEx(thresholded, cv.MORPH_CLOSE, np.ones((3,3), np.uint8), iterations=2)
        # plt.imshow(closed)
        # plt.title("Closed")
        # plt.show()
        open = cv.morphologyEx(closed, cv.MORPH_OPEN, np.ones((5,5), np.uint8), iterations=5)

        #Cut the bottom of the image and make it white
        cut_img = open
        cut_img = cut_img[:-50, :]
        # plt.imshow(cut_img)
        # plt.title("cut_img")
        # plt.show()

        white = np.ones(img.shape[:2], dtype=np.uint8)*255
        white[:-50,:] = cut_img
        final_img = white
        # plt.imshow(final_img)
        # plt.title("Final")
        # plt.show()

        # Find the coords of the center of the biggest blob
        final_img = cv.bitwise_not(final_img) #invert image because connectedComponentsWithStats assumes white as components
        (numLabels, labels, stats, centroids) = cv.connectedComponentsWithStats(final_img.astype(np.uint8))
        # print(numLabels)
        # print(labels)
        # print(stats)
        # print(centroids)
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
            if area > max_blob_area:
                max_blob_area = area
                max_blob = i
                (CxFinal,CyFinal,xFinal,yFinal,hFinal,wFinal) = (cX,cY,x,y,h,w)
            output = final_img
            # cv.rectangle(output, (x, y), (x + w, y + h), (140, 140, 140), 3)
            # cv.circle(output, (int(cX), int(cY)), 4, (140, 140, 140), -1)
            # # connected component ID
            # componentMask = (labels == i).astype("uint8") * 255
            # # show our output image and connected component mask

            # plt.imshow(output)
            # plt.title("output")
            # plt.show()
            # plt.imshow(componentMask)
            # plt.title("components")
            # plt.show()
        return (CxFinal,CyFinal,xFinal,yFinal,hFinal,wFinal)
    
    def convert_to_base_link(self, projected_pose:np.ndarray)->PoseStamped:
        """
        Publish a transform from the camera frame to the map frame
        """
        pose = PoseStamped()
        print(projected_pose)
        pose.header.frame_id = self.arm_cam_frame
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = projected_pose[0]
        pose.pose.position.y = projected_pose[1]
        pose.pose.position.z = projected_pose[2]

        pose.pose.orientation.x = 0.5
        pose.pose.orientation.y = 0.5
        pose.pose.orientation.z = 0.5
        pose.pose.orientation.w = 0.5

        transformed_pose = self.buffer.transform(
            pose, "base_link", rospy.Duration(1.0))

        print(transformed_pose)
        # NOTE: Only for visualization in RViz   
        transformed = TransformStamped()
        transformed.header.frame_id = "base_link"
        transformed.header.stamp = rospy.Time.now()
        transformed.child_frame_id = "arm_cam_debug"
        transformed.transform.translation.x = transformed_pose.pose.position.x
        transformed.transform.translation.y = transformed_pose.pose.position.y
        transformed.transform.translation.z = transformed_pose.pose.position.z
        transformed.transform.rotation.x = transformed_pose.pose.orientation.x
        transformed.transform.rotation.y = transformed_pose.pose.orientation.y
        transformed.transform.rotation.z = transformed_pose.pose.orientation.z
        transformed.transform.rotation.w = transformed_pose.pose.orientation.w
        self.broadcaster.sendTransform(transformed)

        return transformed_pose
    
    def publish_pickup_goal(self, pose:PoseStamped):
        self.pickup_goal_pub.publish(pose)
    
if __name__ == "__main__":
    rospy.init_node("armcam_detection")
    memory_node = armcamDetection()
    rospy.spin()
