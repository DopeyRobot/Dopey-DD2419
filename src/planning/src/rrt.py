#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import math
import random
import numpy as np
import matplotlib.pyplot as plt

# Assumes a map generetad by SLAM in 2D occupancy grid, where 1 is occupied and 0 free.
# The path generated will be a list of 2D coordinates representing the points along the path. 

import yaml

# Load the .yaml file
with open("maptest.yaml", "r") as file:
    map_data = yaml.load(file, Loader=yaml.FullLoader)

# Access the map information
resolution = map_data['map_resolution']
origin = map_data['map_origin']
width = map_data['map_width']
height = map_data['map_height']
map_data = map_data['map_data']

# Read the PGM file
'''with open("maptest.pgm", "r") as f:
    lines = f.readlines()
    map_data = []
    for line in lines[3:]:
        map_data.append([int(x) for x in line.strip().split()])'''

# Convert to a numpy array
map_data = np.array(map_data)

# Save the numpy array to a file
np.save("maptest.npy", map_data)

# Load the map generated using SLAM
map_data = map_data

# Define the start and goal locations
start = [0, 0]
goal = [100, 100]

# Initialize an empty list RRT to store the nodes in the tree.
RRT = [start]

# Define the parameters for the RRT
num_iterations = 1000
step_size = 5

# Generate the RRT, In each iteration, generate a random point x_rand within the map.
for i in range(num_iterations):

    # Radnom point
    x_rand = [random.randint(0, map_data.shape[0]), random.randint(0, map_data.shape[1])]

    # Find the node x_nearest in RRT that is closest to x_rand using the Euclidean distance.
    x_nearest = RRT[0]
    for x in RRT:
        if np.linalg.norm(np.array(x_rand) - np.array(x)) < np.linalg.norm(np.array(x_rand) - np.array(x_nearest)):
            x_nearest = x

    # Calculate a new point x_new by moving from x_nearest towards x_rand by step_size units. (normalized)
    x_new = [0, 0]
    x_new[0] = x_nearest[0] + step_size * (x_rand[0] - x_nearest[0]) / np.linalg.norm(np.array(x_rand) - np.array(x_nearest))
    x_new[1] = x_nearest[1] + step_size * (x_rand[1] - x_nearest[1]) / np.linalg.norm(np.array(x_rand) - np.array(x_nearest))

    # Check if the new point x_new collides with any obstacles in the map. If it does, continue to the next iteration. If it doesn't, add x_new to RRT.
    if not any(np.array(map_data[x_new[0]][x_new[1]]) == 0):
        RRT.append(x_new)

    # Check if the distance between x_new and the goal is less than or equal to step_size.
    if np.linalg.norm(np.array(x_new) - np.array(goal)) <= step_size:
        print("Found goal!")
        RRT.append(goal)
        break

# Generate the path
path = [goal]
x = goal
while x != start:
    for i in range(len(RRT)):
        if np.linalg.norm(np.array(x) - np.array(RRT[i])) <= step_size:
            path.append(RRT[i])
            x = RRT[i]
            break

# Plot the map and the path
#plt.imshow(map_data, cmap='gray')
#plt.plot(start[0], start[1], 'ro', markersize=10)
#plt.plot(goal[0], goal[1], 'go', markersize=10)
#plt.plot(np.array(path)[:,0], np.array(path)[:,1], 'r')
#plt.show()


# Idea to use it with ROS (very unsure about this.....)
# publish the path information as PoseStamped? or should it be twist, or duty cycles.... 

from geometry_msgs.msg import PoseStamped

rospy.init_node('path_publisher')
path_pub = rospy.Publisher('/motor_controller/twist', PoseStamped, queue_size=10)

def path_callback(path):
    for pose in path:
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose.position.x = pose[0]
        pose_stamped.pose.position.y = pose[1]
        pose_stamped.pose.position.z = 0
        pose_stamped.pose.orientation.x = 0
        pose_stamped.pose.orientation.y = 0
        pose_stamped.pose.orientation.z = 0
        pose_stamped.pose.orientation.w = 1
        path_pub.publish(pose_stamped)


rospy.Subscriber('path', path_callback)
rospy.spin()

