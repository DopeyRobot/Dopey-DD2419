#!/usr/bin/env python3

"""
    # {Louis De Oliveira}
    # {student id}
    # {louisdo@kth.se}
"""

# Python standard library
from math import cos, sin, atan2, fabs
from itertools import permutations

# Numpy
import numpy as np

# "Local version" of ROS messages
from geometry_msgs.msg import PoseStamped, Quaternion
from sensor_msgs.msg import LaserScan
from map_msgs.msg import OccupancyGridUpdate

from grid_map import GridMap


class Mapping:
    def __init__(
        self, unknown_space, free_space, c_space, occupied_space, radius, optional=None
    ):
        self.unknown_space = unknown_space
        self.free_space = free_space
        self.c_space = c_space
        self.occupied_space = occupied_space
        self.allowed_values_in_map = {
            "self.unknown_space": self.unknown_space,
            "self.free_space": self.free_space,
            "self.c_space": self.c_space,
            "self.occupied_space": self.occupied_space,
        }
        self.radius = radius
        self.__optional = optional

    def get_yaw(self, q):
        """Returns the Euler yaw from a quaternion.
        :type q: Quaternion
        """
        return atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def raytrace(self, start, end):
        """Returns all cells in the grid map that has been traversed
        from start to end, including start and excluding end.
        start = (x, y) grid map index
        end = (x, y) grid map index
        """
        (start_x, start_y) = start
        (end_x, end_y) = end
        x = start_x
        y = start_y
        (dx, dy) = (fabs(end_x - start_x), fabs(end_y - start_y))
        n = dx + dy
        x_inc = 1
        if end_x <= start_x:
            x_inc = -1
        y_inc = 1
        if end_y <= start_y:
            y_inc = -1
        error = dx - dy
        dx *= 2
        dy *= 2

        traversed = []
        for i in range(0, int(n)):
            traversed.append((int(x), int(y)))

            if error > 0:
                x += x_inc
                error -= dy
            else:
                if error == 0:
                    traversed.append((int(x + x_inc), int(y)))
                y += y_inc
                error += dx

        return traversed

    def add_to_map(self, grid_map, x, y, value):
        """Adds value to index (x, y) in grid_map if index is in bounds.
        Returns weather (x, y) is inside grid_map or not.
        """
        if value not in self.allowed_values_in_map.values():
            raise Exception(
                "{0} is not an allowed value to be added to the map. ".format(value)
                + "Allowed values are: {0}. ".format(self.allowed_values_in_map.keys())
                + "Which can be found in the '__init__' function."
            )

        if self.is_in_bounds(grid_map, x, y):
            grid_map[x, y] = value
            return True
        return False

    def is_in_bounds(self, grid_map, x, y):
        """Returns weather (x, y) is inside grid_map or not."""
        if x >= 0 and x < grid_map.get_width():
            if y >= 0 and y < grid_map.get_height():
                return True
        return False

    def rotate_coords(self, x, y, angle):
        return x * cos(angle) - y * sin(angle), x * sin(angle) + y * cos(angle)

    def update_map(self, grid_map, pose, scan):
        """Updates the grid_map with the data from the laser scan and the pose.

        For E:
            Update the grid_map with self.occupied_space.

            Return the updated grid_map.

            You should use:
                self.occupied_space  # For occupied space

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        For C:
            Update the grid_map with self.occupied_space and self.free_space. Use
            the raytracing function found in this file to calculate free space.

            You should also fill in the update (OccupancyGridUpdate()) found at
            the bottom of this function. It should contain only the rectangle area
            of the grid_map which has been updated.

            Return both the updated grid_map and the update.

            You should use:
                self.occupied_space  # For occupied space
                self.free_space      # For free space

                To calculate the free space you should use the raytracing function
                found in this file.

                You can use the function add_to_map to be sure that you add
                values correctly to the map.

                You can use the function is_in_bounds to check if a coordinate
                is inside the map.

        :type grid_map: GridMap
        :type pose: PoseStamped
        :type scan: LaserScan
        """

        # Current yaw of the robot
        robot_yaw = self.get_yaw(pose.pose.orientation)
        # The origin of the map [m, m, rad]. This is the real-world pose of the
        # cell (0,0) in the map.
        origin = grid_map.get_origin()
        # The map resolution [m/cell]
        resolution = grid_map.get_resolution()

        """
        Fill in your solution here
        Args : gridmap, pose, scan
        """
        robot_position = pose.pose.position
        x_r, y_r = robot_position.x, robot_position.y

        self.xf_list = []
        self.yf_list = []

        angle = scan.angle_min
        for distance in scan.ranges:
            if scan.range_min < distance < scan.range_max:
                x_o_r = distance * cos(angle)
                y_o_r = distance * sin(angle)

                x_o, y_o = self.rotate_coords(x_o_r, y_o_r, robot_yaw)

                x_f = int((x_o + x_r - origin.position.x) / resolution)
                y_f = int((y_o + y_r - origin.position.y) / resolution)

                if self.is_in_bounds(grid_map, x_f, y_f):
                    self.xf_list.append(x_f)
                    self.yf_list.append(y_f)
            angle += scan.angle_increment

        """
        For C only!
        Fill in the update correctly below.
        """
        # Only get the part that has been updated
        update = OccupancyGridUpdate()
        # The minimum x index in 'grid_map' that has been updated
        update.x = min(self.xf_list)
        # The minimum y index in 'grid_map' that has been updated
        update.y = min(self.yf_list)
        # Maximum x index - minimum x index + 1
        update.width = max(self.xf_list) - update.x + 1
        # Maximum y index - minimum y index + 1
        update.height = max(self.yf_list) - update.y + 1
        # The map data inside the rectangle, in row-major order.
        update.data = []

        ix_r = int((x_r - origin.position.x) / resolution)
        iy_r = int((y_r - origin.position.y) / resolution)
        for xf, yf in zip(self.xf_list, self.yf_list):
            traversed = self.raytrace((ix_r, iy_r), (xf, yf))

            for x_t, y_t in traversed:
                self.add_to_map(grid_map, x_t, y_t, self.free_space)

        for xf, yf in zip(self.xf_list, self.yf_list):
            self.add_to_map(grid_map, xf, yf, self.occupied_space)

        for y in range(update.height):
            for x in range(update.width):
                update.data.append(grid_map[x + update.x, y + update.y])
        return grid_map, update

    def inflate_map(self, grid_map):
        """For C only!
        Inflate the map with self.c_space assuming the robot
        has a radius of self.radius.

        Returns the inflated grid_map.

        Inflating the grid_map means that for each self.occupied_space
        you calculate and fill in self.c_space. Make sure to not overwrite
        something that you do not want to.


        You should use:
            self.c_space  # For C space (inflated space).
            self.radius   # To know how much to inflate.

            You can use the function add_to_map to be sure that you add
            values correctly to the map.

            You can use the function is_in_bounds to check if a coordinate
            is inside the map.

        :type grid_map: GridMap
        """

        """
        Fill in your solution here
        """
        r = self.radius
        perm = [
            (i, j)
            for i in range(-r, r + 1)
            for j in range(-r, r + 1)
            if np.sqrt(i * i + j * j) <= r
        ]
        for x in range(grid_map.get_width()):
            for y in range(grid_map.get_height()):
                if grid_map[x, y] == self.occupied_space:
                    for i, j in perm:
                        if grid_map[x + i, y + j] != self.occupied_space:
                            grid_map[x + i, y + j] = self.c_space

        # Return the inflated map
        return grid_map
