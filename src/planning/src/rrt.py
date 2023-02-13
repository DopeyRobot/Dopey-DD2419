#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import random
import numpy as np
import matplotlib.pyplot as plt
from buildmap import map_data


# Define the start and goal locations
start = [0, 0]
goal = [20, 20]

# Initialize an empty list RRT to store the nodes in the tree.
RRT = [start]

# Define the parameters for the RRT
num_iterations = 10
step_size = 5

# Generate the RRT, In each iteration, generate a random point x_rand within the map.
for i in range(num_iterations):
    print(RRT)
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

    x_new_0 = int(round(x_new[0]))
    x_new_1 = int(round(x_new[1]))
    # Check if the new point x_new collides with any obstacles in the map. If it does, continue to the next iteration. If it doesn't, add x_new to RRT.
    if not map_data[x_new_0][x_new_1] == 1:
        RRT.append([x_new_0, x_new_1])

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
            print(x)
            break


print(path)
# Plot the map and the path
plt.imshow(map_data, cmap='gray')
plt.plot(start[0], start[1], 'ro', markersize=10)
plt.plot(goal[0], goal[1], 'go', markersize=10)
plt.plot(np.array(path)[:,0], np.array(path)[:,1], 'r')
plt.show()






