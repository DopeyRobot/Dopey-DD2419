#!/usr/bin/env python3
import rospy
import numpy as np
import tf2_ros
import tf2_geometry_msgs
import math
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped
from random import randint
from std_msgs.msg import Bool

class FrontierExploration:
    def __init__(self):

        rospy.Subscriber('/occupancygrid', OccupancyGrid, self.map_callback)
        self.publish_goal = rospy.Publisher('/send_goal', PoseStamped, queue_size=1)
        self.subcribe_ready_for_path = rospy.Subscriber('/ready_for_new_path', Bool, self.ready_for_path_callback)

        self.buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        rospy.sleep(0.5)
        self.listener = tf2_ros.TransformListener(self.buffer)
        rospy.sleep(0.5)

        self.occupancy_grid = None
        self.ready_for_path = False

        self.run()

    def ready_for_path_callback(self, msg):
        self.ready_for_path = msg.data
        

    def map_callback(self, msg):
        self.map_data = msg
        self.occupancy_grid = np.asarray(self.map_data.data, dtype=np.int8).reshape(self.map_data.info.height, self.map_data.info.width)
        # print("Inside map callback")
        # print(self.occupancy_grid)


    def identify_froniters(self):

        frontier_cells = []
        if self.occupancy_grid is not None:
            for i in range(1, self.occupancy_grid.shape[0]-1):
                for j in range(1, self.occupancy_grid.shape[1]-1):
                    if self.occupancy_grid[i][j] == 0 and np.sum(self.occupancy_grid[i-1:i+2, j-1:j+2]) > 0:
                        x = (j - 0.5) * self.map_data.info.resolution + self.map_data.info.origin.position.x
                        y = (i - 0.5) * self.map_data.info.resolution + self.map_data.info.origin.position.y
                        frontier_cells.append((x, y))

        return frontier_cells
    
        # frontier_cells = []
        # frontier_cells_in_map = []
        # if self.occupancy_grid is not None:
            
        #     for i in range(1, self.occupancy_grid.shape[0]-1):
        #         for j in range(1, self.occupancy_grid.shape[1]-1):
        #             if self.occupancy_grid[i][j] == 0 and np.sum(self.occupancy_grid[i-1:i+2, j-1:j+2]) > 0:
        #                 #print('frontier')

        #                 # Compute row and column indices
        #                 col = int((j - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        #                 row = int((i - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        #                 # Check if row and col indices are within bounds
        #                 if 0 <= row < self.occupancy_grid.shape[0] and 0 <= col < self.occupancy_grid.shape[1]:
        #                     cell = self.occupancy_grid[row][col]
        #                     frontier_cells_in_map.append(cell)
                

        # return frontier_cells_in_map


    def euclidean_distance(self, x1, y1, x2, y2):

        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

    def get_current_position(self):
        base_link_origin = PoseStamped()
        base_link_origin.header.stamp = rospy.Time.now()

        transform_to_map = self.buffer.lookup_transform("map", "base_link", base_link_origin.header.stamp , rospy.Duration(1))  
        baseInMapPose = tf2_geometry_msgs.do_transform_pose(base_link_origin, transform_to_map)

        return baseInMapPose
        

    def distance2frontier(self):
        current_position = self.get_current_position()
        current_x = current_position.pose.position.x
        current_y = current_position.pose.position.y
        frontier_cells = self.identify_froniters()
        # print(frontier_cells)

        distances = []
        for cell in frontier_cells:
            dist = self.euclidean_distance(current_x, current_y, cell[0], cell[1])
            distances.append(dist)

        # closest frontier cell
        if len(distances) != 0:
            min_distance_idx = np.argmin(distances)
            closest_frontier = frontier_cells[min_distance_idx]

            return closest_frontier
        else:
            return None

    def run(self):
        while not rospy.is_shutdown():
            froniter_to_publish = PoseStamped()
            closest_frontier = self.distance2frontier()
            if closest_frontier is not None:
                froniter_to_publish.pose.position.x = closest_frontier[0]*2
                froniter_to_publish.pose.position.y = closest_frontier[1]*2

                if self.ready_for_path:
                    self.publish_goal.publish(froniter_to_publish)



if __name__ == "__main__":
    rospy.init_node('local_frontier_detector')
    FrontierExploration()
    rospy.spin()


