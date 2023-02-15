#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
from open3d import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as o3drh
import numpy as np
from tf2_ros import TransformListener, TransformBroadcaster, Buffer, TransformStamped


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

        self.filter_points()

    def filter_points(self):
        norm_mask = np.linalg.norm(self.points, axis=1) < 0.9
        height_mask = self.points[:, 2] > 0.1
        norm_and_height = np.logical_and(norm_mask, height_mask)
        mean = np.mean(self.points[norm_and_height], axis=0)

        if np.all(mean != np.nan):

            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "camera_color_optical_frame"
            transform.child_frame_id = "object"
            transform.transform.translation.x = mean[0]
            transform.transform.translation.y = mean[1]
            transform.transform.translation.z = mean[2]
            transform.transform.rotation.x = 0
            transform.transform.rotation.y = 0
            transform.transform.rotation.z = 0
            transform.transform.rotation.w = 1

            transformed_pose = self.buffer.transform(
                transform, "map", rospy.Duration(2)
            )

            self.broadcaster.sendTransform(transformed_pose)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("detection")
    Detection()
    rospy.spin()
