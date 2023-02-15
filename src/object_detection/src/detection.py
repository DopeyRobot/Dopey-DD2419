#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from open3d import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as o3drh
import numpy as np
from tf2_ros import TransformListener, TransformBroadcaster, Buffer, TransformStamped
from geometry_msgs.msg import PoseStamped


class Detection:
    def __init__(self):
        self.broadcaster = TransformBroadcaster()
        self.buffer = Buffer(rospy.Duration(1200.0))
        self.listener = TransformListener(self.buffer)
        self.cloud_sub = rospy.Subscriber(
            "/camera/depth/color/points", PointCloud2, self.cloud_callback
        )
        self.rate = rospy.Rate(10)
        self.points = np.array([])
        self.colors = np.array([])

        self.run()

    def cloud_callback(self, msg: PointCloud2):
        # Convert ROS -> Open3D
        cloud = o3drh.rospc_to_o3dpc(msg)

        # Downsample the point cloud to 2 cm
        ds_cloud = cloud.voxel_down_sample(voxel_size=0.02)

        # Convert Open3D -> NumPy
        self.points = np.asarray(ds_cloud.points)
        self.colors = np.asarray(ds_cloud.colors)

        self.filter_points(msg.header.stamp)

    def filter_points(self, stamp):
        norm_mask = np.linalg.norm(self.points, axis=1) < 0.9
        height_mask = self.points[:, 2] > 0.1
        norm_and_height = np.logical_and(norm_mask, height_mask)
        mean = np.mean(self.points[norm_and_height], axis=0)

        if np.all(mean != np.nan):

            pose = PoseStamped()
            pose.header.stamp = stamp
            pose.header.frame_id = "camera_color_optical_frame"
            pose.pose.position.x = mean[0]
            pose.pose.position.y = mean[1]
            pose.pose.position.z = mean[2]
            pose.pose.orientation.x = 0
            pose.pose.orientation.y = 0
            pose.pose.orientation.z = 0
            pose.pose.orientation.w = 1

            transformed_pose = self.buffer.transform(
                pose, "map", rospy.Duration(2)
            )
            t = TransformStamped()
            t.header.stamp = transformed_pose.header.stamp

            t.transform.translation.x = transformed_pose.pose.position.x
            t.transform.translation.y = transformed_pose.pose.position.y
            t.transform.translation.z = transformed_pose.pose.position.z

            t.transform.rotation.x = transformed_pose.pose.orientation.x
            t.transform.rotation.y = transformed_pose.pose.orientation.y
            t.transform.rotation.z = transformed_pose.pose.orientation.z
            t.transform.rotation.w = transformed_pose.pose.orientation.w

            transformed_pose.child_frame_id = "object"
            self.broadcaster.sendTransform(t)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("detection")
    Detection()
    rospy.spin()
