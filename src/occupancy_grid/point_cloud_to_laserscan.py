#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import PointCloud2, LaserScan
from tf2_ros import Buffer, TransformListener
import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as o3drh
import numpy as np


class PointCloudToLaserScan:
    def __init__(self) -> None:
        self.f = 10
        self.sub_pointcloud = rospy.Subscriber(
            "/camera/depth/color/points", PointCloud2, self.cloud_callback
        )
        self.pub_laserscan = rospy.Publisher("/laserscan", LaserScan, queue_size=10)

        self.voxel_size = 0.01
        ## laserscan parameters
        self.min_angle = -1.57
        self.max_angle = 1.57
        self.angle_increment = 0.01
        self.scan_time = 1 / self.f
        self.range_min = 0.1
        self.range_max = 3.0

        self.target_frame = "map"
        self.source_frame = "camera_color_frame"

    def cloud_callback(self, msg):
        laserscan = LaserScan()
        laserscan.header = msg.header
        laserscan.header.frame_id = self.source_frame

        laserscan.angle_min = self.min_angle
        laserscan.angle_max = self.max_angle
        laserscan.angle_increment = self.angle_increment
        laserscan.scan_time = self.scan_time
        laserscan.range_min = self.range_min
        laserscan.range_max = self.range_max

        N_ranges = int((self.max_angle - self.min_angle) / self.angle_increment)
        ranges = np.ones(N_ranges)*self.range_max

        # transform pointcloud to map frame

        # iterate over all points in pointcloud
        cloud = o3drh.rospc_to_o3dpc(msg)

        # Downsample the point cloud to 5 cm
        cloud = cloud.voxel_down_sample(voxel_size=self.voxel_size)

        # Convert Open3D -> NumPy
        points = np.asarray(cloud.points)
        height_req = points[:, 1] < 0.10
        # isnanx = np.isnan(points[:, 0])
        # isnany = np.isnan(points[:, 1])
        # isnanz = np.isnan(points[:, 2])
        # isnan = np.logical_or(isnanx, isnany)
        # isnan = np.logical_or(isnan, isnanz)
        # final_mask = np.logical_and(height_req, np.logical_not(isnan))

        dist_aboveground = points[height_req]  # these will be occupied spaces

        for point in dist_aboveground:
            # get angle of point
            x = point[2]
            y = -point[0]
            angle = np.arctan2(y, x)
            # get range of point
            distance = np.hypot(x, y)
            # print(distance)
            if angle < self.min_angle or angle > self.max_angle:
                continue
            if distance < self.range_min or distance > self.range_max:
                continue

            # get index of distance
            index = int((angle - self.min_angle) / self.angle_increment)

            # check if distance is smaller than current distance
            if distance < ranges[index]:
                ranges[index] = distance
                
        laserscan.ranges = ranges
        self.pub_laserscan.publish(laserscan)


if __name__ == "__main__":
    rospy.init_node("point_cloud_to_laserscan")
    node = PointCloudToLaserScan()
    rospy.spin()
