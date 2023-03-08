#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import PointCloud2
import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as o3drh
import numpy as np
from tf2_ros import TransformListener, TransformBroadcaster, Buffer, TransformStamped
from tf2_geometry_msgs import PoseStamped

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

        # Downsample the point cloud to 2 cm between points
        ds_cloud = cloud.voxel_down_sample(voxel_size=0.02)


        # Convert Open3D -> NumPy
        self.points = np.asarray(ds_cloud.points)
        self.colors = np.asarray(ds_cloud.colors)

        self.filter_points(msg.header.stamp)

    def filter_points(self, stamp):
        norm_mask = np.linalg.norm(self.points, axis=1) < 0.5
        height_mask = self.points[:, 1] < 0.101
        norm_and_height = np.logical_and(norm_mask, height_mask)
        points = self.points[norm_and_height]
        if len(points)>0:
            cloud = o3d.geometry.PointCloud()
            cloud.points = o3d.utility.Vector3dVector(points)
            labels = np.array(cloud.cluster_dbscan(eps=0.02, min_points=10))
            clusters = range(labels.max()+1)

            for cluster in clusters:
                cluster_bool = labels == cluster

                mean = np.mean(points[cluster_bool], axis=0)

                if not np.any(np.isnan(mean)):
                    pose = PoseStamped()
                    pose.header.stamp = stamp
                    pose.header.frame_id = "camera_color_optical_frame"
                    pose.pose.position.x = mean[0]
                    pose.pose.position.y = mean[1]
                    pose.pose.position.z = mean[2]
                    pose.pose.orientation.x = 0.5
                    pose.pose.orientation.y = 0.5
                    pose.pose.orientation.z = 0.5
                    pose.pose.orientation.w = 0.5

                    transformed_pose = self.buffer.transform(
                        pose, "map", rospy.Duration(1)
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

                    t.child_frame_id = f"object{cluster}"
                    self.broadcaster.sendTransform(t)

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("cloud_object_detection")
    Detection()
    rospy.spin()
