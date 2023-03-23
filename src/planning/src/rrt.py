#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import random
import numpy as np
from buildmap import map_data
import matplotlib.pyplot as plt
from typing import List, Tuple
import tf_conversions
import tf2_ros
import tf2_geometry_msgs

class RRTNode:
    def __init__(self, x, y, parent=None) -> None:
        self.x = x
        self.y = y
        self.parent = parent
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
        self.num_iterations = num_iterations
        self.step_size = step_size
        self.n_steps = n_steps
        self.goal_sample_prob = 0.1

        self.pub_path = rospy.Publisher("/path_topic", Path, queue_size=10)
        self.sub_goal = rospy.Subscriber("/send_goal", PoseStamped, self.send_goal_callback)
        self.rate = rospy.Rate(1)

        self.path_msg = Path()
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.frame_id = "map"

        self.fig, self.ax = plt.subplots()

        self.RRT: List[RRTNode] = [self.start]


    def send_goal_callback(self, msg):
        self.goal = [msg.pose.position.x, msg.pose.position.y]

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
        for node in self.RRT:
            if np.linalg.norm(
                np.array([x, y]) - np.array([node.x, node.y])
            ) < np.linalg.norm(
                np.array([x, y]) - np.array([nearest_node.x, nearest_node.y])
            ):
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

    def RRT_step(self, nearest_node: RRTNode, target_x, target_y):
        new_node = RRTNode(nearest_node.x, nearest_node.y, nearest_node)
        for step in range(self.n_steps):
            new_x, new_y = self.move_step(new_node.x, new_node.y, target_x, target_y)
            if self.check_map(new_x, new_y):
                new_node.x = new_x
                new_node.y = new_y
            else:
                print("Ran into obstacle")
                return None

        return new_node

    def generate_RRT(self):
        for i in range(self.num_iterations):
            x_rand, y_rand = self.sample_random()
            nearest_node = self.find_nearest(x_rand, y_rand)
            new_node = self.RRT_step(nearest_node, x_rand, y_rand)
            if new_node is not None:
                self.RRT.append(new_node)
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
            # self.plot_RRT_tree()
            #print(f"done with iteration {i}")

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
