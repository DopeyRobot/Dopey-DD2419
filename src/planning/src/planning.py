#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rrt import path 

# Idea to use it with ROS (very unsure about this.....)
# publish the path information as PoseStamped? or should it be twist, or duty cycles.... 


rospy.init_node('planning')
path_pub = rospy.Publisher('path', Path, queue_size=10)


# Convert the path to a ROS Path message
path_msg = Path()
path_msg.header.frame_id = "map"
for pose in path:
    pose_msg = PoseStamped()
    pose_msg.pose.position.x = pose[0]
    pose_msg.pose.position.y = pose[1]
    path_msg.poses.append(pose_msg)

# Publish the Path message
rate = rospy.Rate(10)

while not rospy.is_shutdown():
    path_pub.publish(path_msg)
    rate.sleep()