#!/usr/bin/env python3
import rospy
from nav_msgs.msg import MapMetaData, OccupancyGrid
import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud2
from open3d import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as o3drh

class OccupancyGrid():
    def __init__(self):
        self.f = 10
        self.rate = rospy.Rate(self.f)
        self.subscriber_pointcloud = rospy.Subscriber("/camera/depth/color/points", PointCloud2, self.cloud_callback)
        self.publisher_occupancygrid = rospy.Publisher("/occupancygrid", OccupancyGrid, queue_size = 10)
        self.out_msg = PointCloud2()
    def cloud_callback(self, pointcloudmsg):
        # Convert ROS -> Open3D
        cloud = o3drh.rospc_to_o3dpc(pointcloudmsg)

        # Downsample the point cloud to 5 cm
        ds_cloud = cloud.voxel_down_sample(voxel_size=0.05)

        # Convert Open3D -> NumPy
        points = np.asarray(cloud.points)
        # print(f"points: {points[0]}\n")
        colors = np.asarray(cloud.colors)
        # print("colors", colors)

        dist_req1 = points[:, 2] <= 0.9
        dist_req2 = points[:,1] >= 0
        dist_index = np.logical_and(dist_req1, dist_req2)

        # Convert Open3D -> ROS
        out_msg = o3drh.o3dpc_to_rospc(ds_cloud)
        out_msg.header = pointcloudmsg.header

    def run(self):
        while not rospy.is_shutdown():
            

