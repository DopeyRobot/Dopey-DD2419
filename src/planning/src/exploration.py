#!/usr/bin/env python3
import rospy
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Point, PoseStamped
from random import randint
import numpy as np
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
import math
from std_msgs.msg import Bool


class FrontierExploration:
    def __init__(self):

        rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.publish_goal = rospy.Publisher('/send_goal', PoseStamped, queue_size=1)
        self.subcribe_ready_for_path = rospy.Subscriber('/ready_for_new_path', Bool, self.ready_for_path_callback)

        self.buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.listener = tf2_ros.TransformListener(self.buffer)

        self.occupancy_grid = None
        self.ready_for_path = None

    def ready_for_path_callback(self, msg):
        self.ready_for_path = msg.data
        

    def map_callback(self, msg):
        occupancy_grid = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.occupancy_grid = occupancy_grid


    def identify_froniters(self):
        frontier_cells = []

        for i in range(1, self.occupancy_grid.shape[0]-1):
            for j in range(1, self.occupancy_grid.shape[1]-1):
                if self.occupancy_grid[i][j] == 0 and np.sum(self.occupancy_grid[i-1:i+2, j-1:j+2]) > 0:
                    frontier_cells.append((i, j))

        return frontier_cells

    def euclidean_distance(self, x1, y1, x2, y2):

        return math.sqrt((x2-x1)**2 + (y2-y1)**2)

    def get_current_position(self):
        robot_pose = PoseStamped()
        robot_pose.pose.position.x = 0
        robot_pose.pose.position.y = 0
        robot_pose.pose.position.z = 0
        robot_pose.header.frame_id = 'base_link'
        robot_pose.header.stamp = rospy.Time.now()

        try:
            transform_to_map = self.buffer.lookup_transform("map", robot_pose.header.frame_id, robot_pose.header.stamp , rospy.Duration(1))           
            current_position = tf2_geometry_msgs.do_transform_pose(robot_pose, transform_to_map)
        except:
            pass
        
        return current_position
        

    def distance2frontier(self):
        current_position = self.get_current_position()
        current_x = current_position.pose.position.x
        current_y = current_position.pose.position.y
        frontier_cells = self.identify_froniters()

        distances = []
        for cell in frontier_cells:
            dist = self.euclidean_distance(current_x, current_y, cell[0], cell[1])
            distances.append(dist)

        # closest frontier cell
        min_distance_idx = np.argmin(distances)
        closest_frontier = frontier_cells[min_distance_idx]

        return closest_frontier

    def publish_closest_frontier(self):
        froniter_to_publish = PoseStamped()
        froniter_to_publish.pose.position.x = self.distance2frontier()[0]
        froniter_to_publish.pose.position.y = self.distance2frontier()[1]

        if self.ready_for_path:
            self.publish_goal.publish(froniter_to_publish)



if __name__ == "__main__":
    rospy.init_node('local_frontier_detector')
    rospy.spin()


