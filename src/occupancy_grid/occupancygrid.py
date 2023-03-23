#!/usr/bin/env python3
import rospy
from nav_msgs.msg import MapMetaData, OccupancyGrid
from geometry_msgs.msg import Pose, PolygonStamped
import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import LaserScan
import open3d as o3d
from open3d_ros_helper import open3d_ros_helper as o3drh
import tf2_ros
import tf2_geometry_msgs


class Occupancygrid:
    def __init__(self):
        self.f = 10
        self.rate = rospy.Rate(self.f)

        self.publisher_occupancygrid = rospy.Publisher(
            "/occupancygrid", OccupancyGrid, queue_size=10
        )
        self.sub_laserscan = rospy.Subscriber(
            "/laserscan", LaserScan, self.scan_callback
        )

        self.sub_workspace = rospy.Subscriber(
            "/workspace", PolygonStamped, self.workspace_callback
        )
        self.laserscan = LaserScan()

        self.cells_width = 200  # number of cells for width : x?
        self.resolution = 5 / self.cells_width  # m/cell
        self.cells_height = 200  # number of cells for height : y ?

        self.vertices_df = pd.read_csv(
            "/home/robot/dd2419_ws/src/occupancy_grid/example_workspace.tsv", sep="\t"
        )  # subscribe from workspace topic istead so only call on
        # self.vertices_df = pd.read_csv("example_workspace.tsv", sep="\t")
        self.vertices = self.vertices_df.values
        print(
            f"vertices: {self.vertices}, x min:{min(self.vertices[:,0])}, xmax: {max(self.vertices[:,0])}, y min: {min(self.vertices[:, 1])}, y max : {max(self.vertices[:,1])}"
        )
        # self.xx_yy = [min(self.vertices[:,0]), max(self.vertices[:,0]), min(self.vertices[:,1]), max(self.vertices[:,1])]
        self.x_low = min(self.vertices[:, 0])
        self.x_high = max(self.vertices[:, 0])
        self.y_low = min(self.vertices[:, 1])
        self.y_high = max(self.vertices[:, 1])
        self.x_n = 200  # number of cells in the x-direction for width
        self.y_n = 200  # number of cells in the y-direction for height
        self.grid = np.zeros((self.x_n, self.y_n))
        # self.vertices_list = np.append(self.vertices, [self.vertices[0]], axis = 0)

        self.buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.source_frame = "camera"
        self.target_frame = "map"
        self.timeout = rospy.Duration(5)

        self.pcd = o3d.geometry.PointCloud()

        self.run()

    def get_i_index(self, x):
        index = math.floor((x - self.x_low) * self.x_n / (self.x_high - self.x_low))
        if index < 0:
            index = 0
        elif index > (self.x_n - 1):
            index = self.x_n - 1
        return index

    def get_j_index(self, y):
        index = math.floor((y - self.y_low) * self.y_n / (self.y_high - self.y_low))
        if index < 0:
            index = 0
        elif index > (self.y_n - 1):
            index = self.y_n - 1
        return index

    def get_x_pos(self, i):
        step = (self.x_high - self.x_low) / self.x_n
        x_pos = (
            self.x_low + step * i + step / 2
        )  # added step / 2 so that; the coordinate is in the middle of the cell
        return x_pos

    def get_y_pos(self, j):
        step = (self.y_high - self.y_low) / self.y_n
        y_pos = (
            self.y_low + step * j + step / 2
        )  # added step/2 so that the coordinate is in the middle of the cells
        return y_pos

    def incrementer(self, x, y, direction):
        # x and y are the current position of the robot in discrete values
        dx = np.cos(direction)
        dy = np.sin(direction)

        x += dx
        y += dy
        return [int(x), int(y)]

    def update_map(self):
        pass

    def scan_callback(self, msg):
        self.laserscan = msg
        self.update_map()
        # for x, y in zip(*coordinate_list):
        #     obstacle_pos_dicrete_space = [self.get_i_index(x), self.get_j_index(y)]
        #     delta_y = obstacle_pos_dicrete_space[1] - robot_pos_disc_space[1]
        #     delta_x = obstacle_pos_dicrete_space[0] - robot_pos_disc_space[0]
        #     bounded = True
        #     while bounded:
        #         # FREE SPACE
        #         direction = np.arctan2(delta_y / delta_x)
        #         robot_explore_pos = self.incrementer(
        #             robot_pos_disc_space[0], robot_pos_disc_space[1], direction
        #         )
        #         self.grid[robot_explore_pos[0], robot_explore_pos[1]] = 0  # free space

        #         bounded = robot_explore_pos != obstacle_pos_dicrete_space

        #     # OCCUPIED SPACE
        #     self.grid[self.get_i_index(x), self.get_j_index(y)] = -1  # occupied
        # # Unknown

        # Free space

    def workspace_callback(self, msg):
        self.vertices = msg.polygon.points
        self.x_low = min(self.vertices[:, 0])
        self.x_high = max(self.vertices[:, 0])
        self.y_low = min(self.vertices[:, 1])
        self.y_high = max(self.vertices[:, 1])

    def run(self):
        while not rospy.is_shutdown():
            metadata = MapMetaData()
            # metadata.map_load_time =
            # metadata.origin =
            metadata.resolution = self.resolution
            metadata.width = self.cells_width
            metadata.height = self.cells_height

            occupancygrid_data = OccupancyGrid()
            occupancygrid_data.info = metadata
            # occupancygrid_data.data =

            self.publisher_occupancygrid.publish(occupancygrid_data)

            self.rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("occupancygrid")
        Occupancygrid()
        rospy.spin()
    except rospy.ROSInternalException:
        pass
