#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Path
import random
import numpy as np
from buildmap import map_data


class RRTPlanner:
    def __init__(self, start, goal, num_iterations=10, step_size=5):
        self.start = start
        self.goal = goal
        self.num_iterations = num_iterations
        self.step_size = step_size

        self.path_msg = Path()
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.frame_id = "map"

        self.RRT = [self.start]

    def generate_RRT(self):
        for i in range(self.num_iterations):
            x_rand = [random.randint(0, map_data.shape[0]), random.randint(0, map_data.shape[1])]
            x_nearest = self.RRT[0]
            for x in self.RRT:
                if np.linalg.norm(np.array(x_rand) - np.array(x)) < np.linalg.norm(
                        np.array(x_rand) - np.array(x_nearest)):
                    x_nearest = x
            x_new = [0, 0]
            x_new[0] = x_nearest[0] + self.step_size * (
                    x_rand[0] - x_nearest[0]) / np.linalg.norm(np.array(x_rand) - np.array(x_nearest))
            x_new[1] = x_nearest[1] + self.step_size * (
                    x_rand[1] - x_nearest[1]) / np.linalg.norm(np.array(x_rand) - np.array(x_nearest))
            x_new_0 = int(round(x_new[0]))
            x_new_1 = int(round(x_new[1]))
            if not map_data[x_new_0][x_new_1] == 1:
                self.RRT.append([x_new_0, x_new_1])
            if np.linalg.norm(np.array(x_new) - np.array(self.goal)) <= self.step_size:
                print("Found goal!")
                self.RRT.append(self.goal)
                break

    def generate_path(self):
        for node in self.RRT:
            pose_stamped = PoseStamped()
            pose_stamped.header = self.path_msg.header
            pose_stamped.pose.position.x = node[0]
            pose_stamped.pose.position.y = node[1]
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            self.path_msg.poses.append(pose_stamped)
        print(self.path_msg)

    def publish_path(self):
        pub = rospy.Publisher('/path_topic', Path, queue_size=10)
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():

            rate.sleep()
            if self.path_msg.poses[len(self.path_msg.poses)-1].pose.position.x == self.goal[0] and self.path_msg.poses[len(self.path_msg.poses)-1].pose.position.y == self.goal[1]:
                pub.publish(self.path_msg)
                print('hej')
                break


if __name__ == '__main__':
    rospy.init_node('rrt')
    start = [0, 0]
    goal = [10, 10]
    planner = RRTPlanner(start, goal, num_iterations=10, step_size=5)
    planner.generate_RRT()
    planner.generate_path()
    planner.publish_path()
