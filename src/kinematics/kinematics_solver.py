#!/usr/bin/env python3
import numpy as np
import rospy
import tf2_ros as tf2
from sensor_msgs.msg import JointState

class KinematicsSolver:

    def __init__(self) -> None:
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.join_state_callback)
        self.cur_joint_state = JointState()
        self.verbose = False
        self.DH_matrix = self.get_DH_matrix()

    def joint_state_callback(self, msg):
        if self.verbose:
            rospy.loginfo("Received new joint state")
        self.cur_joint_state = msg

    def get_DH_matrix(self):

        matrix = np.array([
            [],
            [],
            [],
            [],
            []
        ])
