#!/usr/bin/env python3
import rospy
import random
import numpy as np
import tf_conversions
import tf2_ros
import tf2_geometry_msgs
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from buildmap import map_data
from typing import List, Tuple
import math

class RRTNode:
    def __init__(self, x, y, cost=0, parent=None, child=None) -> None:
        self.x = x
        self.y = y
        self.cost = cost
        self.parent = parent
        self.child = child

        self.buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.listener = tf2_ros.TransformListener(self.buffer)

    def set_parent(self, parent: "RRTNode"):
        self.parent = parent

    def get_parent(self):
        return self.parent
    
    def get_start(self):                
        
        robot_pose = PoseStamped()
        robot_pose.pose.position.x = 0
        robot_pose.pose.position.y = 0
        robot_pose.pose.position.z = 0
        robot_pose.header.frame_id = 'base_link'
        robot_pose.header.stamp = rospy.Time.now()

        try:
            transform_to_map = self.buffer.lookup_transform("map", robot_pose.header.frame_id, robot_pose.header.stamp , rospy.Duration(1))           
            start_pose = tf2_geometry_msgs.do_transform_pose(robot_pose, transform_to_map)

        except:
            start_pose = [0, 0]
        
        return start_pose
        

class RRTPlanner:
    def __init__(self, start, goal, num_iterations=100, step_size=2, n_steps=1):
        
        self.start = RRTNode(start[0], start[1])

        try:
            self.start.x = self.start.get_start().pose.position.x
            self.start.y = self.start.get_start().pose.position.y
        except:
            self.start.x = self.start.get_start()[0]
            self.start.y = self.start.get_start()[1]
        
  
        self.goal = goal

        self.map_data = None
        self.occupancy_grid = None

        self.num_iterations = num_iterations
        self.step_size = step_size
        self.n_steps = n_steps
        self.goal_sample_prob = 0.1

        self.pub_path = rospy.Publisher("/path_topic", Path, queue_size=10)
        self.sub_goal = rospy.Subscriber("/send_goal", PoseStamped, self.send_goal_callback)
        self.sub_map = rospy.Subscriber('/occupancygrid', OccupancyGrid, self.get_map_callback)
        self.rate = rospy.Rate(1)

        self.path_msg = Path()
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.frame_id = "map"

        self.fig, self.ax = plt.subplots()

        self.RRT: List[RRTNode] = [self.start]

    def get_map_callback(self, msg):
        self.map_data = msg
        self.occupancy_grid = np.asarray(self.map_data.data, dtype=np.int8).reshape(self.map_data.info.height, self.map_data.info.width)

    def send_goal_callback(self, msg):
        self.goal = [msg.pose.position.x, msg.pose.position.y]


    def rewire(self, new_node):
        for node in self.RRT:
            if node == new_node or node.parent == new_node:
                continue
            new_cost = new_node.cost + math.sqrt((node.x - new_node.x) ** 2 + (node.y - new_node.y) ** 2)
            if new_cost < node.cost:
                node.parent = new_node
                node.cost = new_cost
                for child in node.children:
                    self.rewire(child)
                # we only need to rewire once per node, so we can break the loop
                break

    def sample_random(self) -> Tuple[int]:
        if random.random() > self.goal_sample_prob:
            return (
                np.random.uniform(0, map_data.shape[0]),
                np.random.uniform(0, map_data.shape[1]),
            )
        else:
            return self.goal

    def find_nearest(self, x, y) -> RRTNode:
        nearest_node = self.RRT[0]
        min_cost = float('inf')
        for node in self.RRT:
            cost = node.cost + np.linalg.norm(
                np.array([x, y]) - np.array([node.x, node.y])
            )
            if cost < min_cost:
                min_cost = cost
                nearest_node = node
        return nearest_node

    def move_step(self, from_x, from_y, to_x, to_y) -> tuple:
        dir = np.array([to_x - from_x, to_y - from_y])
        dir = dir / np.linalg.norm(dir)
        new_pos = np.array([from_x, from_y])
        new_pos = new_pos + self.step_size * dir

        return new_pos[0], new_pos[1]

    def check_map(self, x, y) -> bool:
        return True
        # # Check bounds   
        # if x < self.map_data.info.origin.position.x or y < self.map_data.info.origin.position.y:
        #     return False
        # if x >= self.map_data.info.origin.position.x + self.map_data.info.width * self.map_data.info.resolution or y >= self.map_data.info.origin.position.y + self.map_data.info.height * self.map_data.info.resolution:
        #     return False

        # # Convert the (x,y) coordinate to a grid cell index
        # col = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        # row = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        # value = self.occupancy_grid[row][col]
        # if value == 0:
        #     # Free
        #     return True
        # elif value == 1:
        #     # Obstacle
        #     return False
        # else:
        #     return True
        #     # For now, its unknown..

    def RRT_step(self, nearest_node: RRTNode, target_x, target_y):
        new_node = RRTNode(nearest_node.x, nearest_node.y, nearest_node.cost, nearest_node.parent, nearest_node.child)
        for step in range(self.n_steps):
            new_x, new_y = self.move_step(new_node.x, new_node.y, target_x, target_y)
            if self.check_map(new_x, new_y):
                new_node.x = new_x
                new_node.y = new_y
            else:
                print("Ran into obstacle")
                return None

        distance = np.linalg.norm(np.array([new_node.x, new_node.y]) - np.array([nearest_node.x, nearest_node.y]))
        cost = nearest_node.cost + distance
        new_node.cost = cost

        return new_node

    def generate_RRT(self):
        print(self.goal)
        for i in range(self.num_iterations):
            x_rand, y_rand = self.sample_random()
            nearest_node = self.find_nearest(x_rand, y_rand)
            new_node = self.RRT_step(nearest_node, x_rand, y_rand)
            if new_node is not None:
                self.RRT.append(new_node)
                self.rewire(new_node) 
                if (
                    np.linalg.norm(
                        np.array([new_node.x, new_node.y]) - np.array(self.goal)
                    )
                    <= self.step_size
                ):
                    print("Found goal!")
                    goal_node = RRTNode(self.goal[0], self.goal[1], new_node)
                    self.RRT.append(goal_node)
                    break


    def generate_path(self):
        current_node = self.RRT[-1]
        while current_node.parent is not None:
            pose_stamped = PoseStamped()
            pose_stamped.header = self.path_msg.header
            pose_stamped.pose.position.x = current_node.x
            pose_stamped.pose.position.y = current_node.y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            self.path_msg.poses.append(pose_stamped)
            current_node = current_node.get_parent()
        self.path_msg.poses = self.path_msg.poses[::-1]

        
        for i, pose in enumerate(self.path_msg.poses):
            next_pose = None
            try:
                next_pose = self.path_msg.poses[i + 1]
            except:
                pass
            if next_pose is not None:
                dy = next_pose.pose.position.y - pose.pose.position.y
                dx = next_pose.pose.position.x - pose.pose.position.x

                orientation = np.arctan2(dy, dx)
                #print(orientation)
                quaternian = tf_conversions.transformations.quaternion_from_euler(0,0,orientation)

                pose.pose.orientation.w = quaternian[3]
                pose.pose.orientation.x = quaternian[0]
                pose.pose.orientation.y = quaternian[1]
                pose.pose.orientation.z = quaternian[2]

    def publish_path(self):
        self.pub_path.publish(self.path_msg)
        
    
    def plot_RRT_tree(self):
        self.fig, self.ax = plt.subplots()
        self.ax.scatter(self.goal[0], self.goal[1], color="red", s=10)
        for i, node in enumerate(self.RRT):

            self.ax.scatter(node.x, node.y, color="blue", s=10)

            parent = node.parent
            if parent is None:
                continue

            self.ax.plot(
                [node.x, parent.x], [node.y, parent.y], color="blue", alpha=0.3
            )

        plt.show()


if __name__ == "__main__":
    rospy.init_node("rrt")
    start = [0, 0]
    goal = [0, 0]
    planner = RRTPlanner(start, goal, num_iterations=1000, step_size=0.09)
    planner.generate_RRT()
    planner.generate_path()
    planner.plot_RRT_tree()
    planner.publish_path()
    rospy.spin()

    # FIX: Tree = Dict, where the parent is the ID and value is the rrt node for the parent.
