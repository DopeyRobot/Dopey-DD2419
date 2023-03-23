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

        self.voxel_size = 0.05
        ## laserscan parameters
        self.min_angle = -1.57
        self.max_angle = 1.57
        self.angle_increment = 0.01
        self.scan_time = 1 / self.f
        self.range_min = 0.1
        self.range_max = 1.5

        self.target_frame = "map"
        self.source_frame = "camera_color_optical_frame"

    def cloud_callback(self, msg):
        laserscan = LaserScan()
        laserscan.header = msg.header

        laserscan.angle_min = self.min_angle
        laserscan.angle_max = self.max_angle
        laserscan.angle_increment = self.angle_increment
        laserscan.scan_time = self.scan_time
        laserscan.range_min = self.range_min
        laserscan.range_max = self.range_max

        N_ranges = int((self.max_angle - self.min_angle) / self.angle_increment)
        ranges = np.zeros(N_ranges)

        # transform pointcloud to map frame

        # iterate over all points in pointcloud
        cloud = o3drh.rospc_to_o3dpc(msg)

        # Downsample the point cloud to 5 cm
        cloud = cloud.voxel_down_sample(voxel_size=self.voxel_size)

        # Convert Open3D -> NumPy
        points = np.asarray(cloud.points)
        height_req = points[:, 1] < 0.10
        isnanx = np.isnan(points[:, 0])
        isnany = np.isnan(points[:, 1])
        isnanz = np.isnan(points[:, 2])
        isnan = np.logical_or(isnanx, isnany)
        isnan = np.logical_or(isnan, isnanz)
        final_mask = np.logical_and(height_req, np.logical_not(isnan))

        dist_aboveground = points[final_mask]  # these will be occupied spaces

        for point in dist_aboveground:
            # get angle of point
            x = point[0]
            y = point[2]
            angle = np.arctan2(y, x)
            # get range of point
            range = np.hypot(x, y)

            if angle < self.min_angle or angle > self.max_angle:
                continue
            if range < self.range_min or range > self.range_max:
                continue

            # get index of range
            index = int((angle - self.min_angle) / self.angle_increment)

            # check if range is smaller than current range
            if range < ranges[index]:
                ranges[index] = range

        laserscan.ranges = ranges
        self.pub_laserscan.publish(laserscan)


if __name__ == "__main__":
    rospy.init_node("point_cloud_to_laserscan")
    node = PointCloudToLaserScan()
    rospy.spin()
