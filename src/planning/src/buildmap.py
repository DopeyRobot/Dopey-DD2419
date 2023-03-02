#!/usr/bin/env python3
import yaml
import cv2
import numpy as np
import sys
np.set_printoptions(threshold=sys.maxsize)

# Load the yaml file
with open("maptest.yaml", 'r') as stream:
    data = yaml.safe_load(stream)
    resolution = data['resolution']
    origin = data['origin']
    size = data['size']

# Load the pgm image
img = cv2.imread("maptest.pgm", cv2.IMREAD_GRAYSCALE)

# Create an empty numpy array to store the map data
map_data = np.zeros((size[1], size[0]))

# Fill in the map data
for i in range(map_data.shape[0]):
    for j in range(map_data.shape[1]):
        map_data[i, j] = img[i, j] * resolution

# Save the map data as a .npy file
np.save("map.npy", map_data)

# Threshold the map data to create an occupancy grid
occupied_threshold = 10.26
map_data[map_data <= occupied_threshold] = 0
map_data[map_data > occupied_threshold] = 1


        
