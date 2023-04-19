#!/usr/bin/env python3
import rospy
import numpy as np
from typing import List
from tf2_geometry_msgs import PoseStamped
from tf2_ros import (
    TransformBroadcaster,
    Buffer,
    TransformListener,
    TransformStamped,
)
from sensor_msgs.msg import LaserScan
from bounding_box_detection.srv import instanceNames
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler


class BoxNode:
    def __init__(self) -> None:
        self.laser_scan_sub = rospy.Subscriber(
            "/scan", LaserScan, self.laser_scan_callback
        )

        self.marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size=2)
        self.scan: LaserScan = rospy.wait_for_message("/scan", LaserScan, timeout=5.0)
        self.reference_frame = "map"
        self.camera_frame_id = self.scan.header.frame_id  # "camera_color_optical_frame"
        self.buffer = Buffer(rospy.Duration(1200))
        self.listener = TransformListener(self.buffer)
        self.broadcaster = TransformBroadcaster()
        self.memory_list_srv = rospy.ServiceProxy("/instances_in_LTM", instanceNames)
        self.distance_threshold = (
            0.35  # distance between box and obstacle to still be considered a bo
        )
        self.delta = 0.1  # distance between two points in the laser scan to still be considered part of the box
        self.run()
        

    def laser_scan_callback(self, msg: LaserScan) -> None:
        self.scan = msg

    def get_boxes_from_memory(self) -> List[str]:
        """Get all boxes in the memory list as a list of python strings"""
        ros_list = self.memory_list_srv().instances
        return [name.data for name in ros_list if "box" in name.data.lower()]

    def get_box_pose(self, frame_id: str) -> TransformStamped:
        pose = PoseStamped()
        try:
            transform = self.buffer.lookup_transform(
                self.camera_frame_id, frame_id, rospy.Time(0)
            )
            pose.header.frame_id = transform.header.frame_id
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation.x = transform.transform.rotation.x
            pose.pose.orientation.y = transform.transform.rotation.y
            pose.pose.orientation.z = transform.transform.rotation.z
            pose.pose.orientation.w = transform.transform.rotation.w
            pose.header.stamp = transform.header.stamp
            return pose
        except Exception as e:
            rospy.logerr(e)
            return None

    def get_obstacles_from_laserscan(self):
        obstacle_map_poses = []
        # try:
        #     transform_camera2map = self.buffer.lookup_transform(
        #         self.reference_frame, "camera_depth_frame", rospy.Time(0)
        #     )  # Is the order of source and target frame correct?

        # except:
        #     print("could not find transform for box")
        #     return

        # look through the laser scan and find the points that are in the box
        N = int(
            (self.scan.angle_max - self.scan.angle_min)
            / self.scan.angle_increment
        )
        angles = np.linspace(self.scan.angle_min, self.scan.angle_max, N)
        for dist, angle in list(zip(self.scan.ranges, angles))[::10]:
            if np.isnan(dist):
                continue
            distancePoseStamped = PoseStamped()
            distancePoseStamped.pose.position.x = dist * np.cos(
                angle
            )  # these need to be rotated into the map frame
            distancePoseStamped.pose.position.y = dist * np.sin(angle)
            distancePoseStamped.header.frame_id = "camera_depth_frame"
            # TODO: check if the transform is correct and if we can use All the time the camera reference frame or if we need the map frame for anything
            obstacle_map_poses.append(
                distancePoseStamped
            )  # tf2_geometry_msgs.do_transform_pose(distancePoseStamped, transform_camera2map) #continuous coordinates
        return obstacle_map_poses

    def run(self) -> None:
        """Main loop of the node"""
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            # Get all boxes in the memory list
            boxes = self.get_boxes_from_memory()
            # Get the list of the poses of the boxes
            box_poses = [self.get_box_pose(box) for box in boxes]
            # get obstacles in laser scan
            obstacle_poses = self.get_obstacles_from_laserscan()
            # check if any of the box_poses coincides with any of the obstacle_poses if so save the index
            for box_pose in box_poses:
                matching_pose_idx = self.find_closest_obstacle(box_pose, obstacle_poses)
                self.measure_box(obstacle_poses, matching_pose_idx)

    def find_closest_obstacle(self, box, obstacles: List[PoseStamped]) -> List[str]:
        """Checks if there is an instance in the long term memory that is close enough
        to the position we want to add and return the name of the closest one"""
        idx_closest_pt = 0
        dist = float("inf")
        prev_dist = float("inf")
        rospy.loginfo(len(obstacles))
        for i in range(len(obstacles)):
            dist = self.get_distance(box, obstacles[i])
            rospy.loginfo(dist)

            if dist < self.distance_threshold:
                if dist < prev_dist:
                    prev_dist = dist
                    idx_closest_pt = i
                    rospy.loginfo(idx_closest_pt)
        if idx_closest_pt != 0:
            return idx_closest_pt
        else:
            return None

    def get_distance(self, pose1, pose2) -> float:
        """Returns the distance between two poses"""
        return np.sqrt(
            (pose1.pose.position.x - pose2.pose.position.x) ** 2
            + (pose1.pose.position.y - pose2.pose.position.y) ** 2
            + (pose1.pose.position.z - pose2.pose.position.z) ** 2
        )

    def measure_box(self, obstacle_poses, idx):
        right_end = PoseStamped()
        # Measure the right side of the box
        current_pt = obstacle_poses[idx]
        for obstacle in obstacle_poses[idx:]:
            # TODO: CHECK which direction the front for the laserscan !!!!!!!!!!!!!!!!!!!!
            # This code asumes that "front" is the direction of the positive y axis in the camera frame.
            if abs(current_pt.pose.position.x - obstacle.pose.position.x) > self.delta:
                right_end = obstacle
                break
        # Measure the left side of the box
        left_end = PoseStamped()
        for obstacle in obstacle_poses[idx::-1]:
            # TODO: CHECK which direction the front for the laserscan !!!!!!!!!!!!!!!!!!!!
            if abs(obstacle.pose.position.x - current_pt.pose.position.x) > self.delta:
                left_end = obstacle
                break
        # TODO: Implement safety check to make sure a left and right end were found

        # TODO: maybe there is a smarter implementation to find the size of the box and compare with the real world size

        # find corner(closest point to the camera) in case box is in corner view
        # (i.e. if the camera can see two sides of the box at the same time)
        box_corner = PoseStamped()
        previous_closest_point = float("inf")
        for obstacle in obstacle_poses:
            # find the point that is closest to to the camera frame origin
            if abs(obstacle.pose.position.y) < abs(previous_closest_point):
                box_corner = obstacle

        # position of the center of the box (where the marker center will be placed)
        box_center = PoseStamped()
        box_center.pose.position.x = (
            left_end.pose.position.x + right_end.pose.position.x
        ) / 2  # this is in map frame and should be in camera depth frame
        box_center.pose.position.y = (
            left_end.pose.position.y + right_end.pose.position.y
        ) / 2
        box_center.pose.position.z = 0.0

        oposite_side = abs(box_corner.pose.position.y - right_end.pose.position.y)
        adjacent_side = abs(box_corner.pose.position.x - right_end.pose.position.x)
        orientation = np.arctan2(oposite_side, adjacent_side)
        q = quaternion_from_euler(0, 0, orientation)

        # if box in side/flat view (i.e. the camera can only see one side of the box)
        if (
            abs(box_corner.pose.position.y - left_end.pose.position.y) < 0.2
            and abs(box_corner.pose.position.y - right_end.pose.position.y) < 0.2
        ):
            side_ln = abs(right_end.pose.position.y - left_end.pose.position.y)
            # check if we are seeing the long side or the short side of the box
            diff_to_16cm = abs(side_ln - 16)
            diff_to_24cm = abs(side_ln - 24)
            if diff_to_16cm < diff_to_24cm:
                box_center.pose.position.y = box_corner.pose.position.y + 8
                q = quaternion_from_euler(0, 0, 0)
            else:
                box_center.pose.position.y = box_corner.pose.position.y + 12
                q = quaternion_from_euler(0, 0, np.pi / 2)

        box_center.pose.orientation.x = q[0]
        box_center.pose.orientation.y = q[1]
        box_center.pose.orientation.z = q[2]
        box_center.pose.orientation.w = q[3]

        self.marker_pub.publish(self.create_marker(box_center))

    def create_marker(
        self,
        pose: PoseStamped,
    ) -> Marker:
        marker = Marker()
        marker.header.frame_id = "camera_depth_frame"
        marker.header.stamp = rospy.Time.now()

        # set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
        marker.type = 1
        marker.id = 0

        # Set the scale of the marker
        marker.scale.x = 16.0
        marker.scale.y = 24.0
        marker.scale.z = 8.0

        # Set the color
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Set the pose of the marker
        marker.pose.position.x = pose.pose.position.x
        marker.pose.position.y = pose.pose.position.y
        marker.pose.position.z = pose.pose.position.z
        marker.pose.orientation.x = pose.pose.orientation.x
        marker.pose.orientation.y = pose.pose.orientation.y
        marker.pose.orientation.z = pose.pose.orientation.z
        marker.pose.orientation.w = pose.pose.orientation.w
        return marker

    # For all boxes in the memory list check if they are in the laser scan
    # (if they have a similar position in the laser scan)
    # get the closest one?
    # if so, save the index maybe and start "box meassurement routine":
    # go to the next point and check delta between the two points.
    # if the delta is too big, end of the box.
    # get the frame id from subscriber and then look up the position in the tf tree
    # self.buffer.lookup_transform("id", "map", rospy.Time(0))
    # get the position of the laser in through the tf tree

    # For all intances in the memory list check if they are in the laser scan
    # (if they have a similar position in the laser scan)
    # get the closest one?
    # if so, save the index maybe and start "box meassurement routine":
    # go to the next point and check delta between the two points.
    # if the delta is too big, end of the box.


if __name__ == "__main__":
    rospy.init_node("box_node")
    node = BoxNode()
    rospy.spin()
