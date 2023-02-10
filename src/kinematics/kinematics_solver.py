#!/usr/bin/env python3
import numpy as np
import rospy
import tf2_ros as tf2
from sensor_msgs.msg import JointState
from pose_service import JointData

# in meters
L12 = 18e-3
L23 = 100e-3
L34 = 97e-3
L45 = 55e-3


class KinematicsSolver:
    def __init__(self) -> None:
        self.joint_state_sub = rospy.Subscriber(
            "/joint_states", JointState, self.join_state_callback
        )
        self.cur_joint_state = JointState()
        self.verbose = False
        self.DH_matrix = self.get_DH_matrix()

    def joint_state_callback(self, msg):
        if self.verbose:
            rospy.loginfo("Received new joint state")
        self.cur_joint_state = msg

    def get_DH_matrix(self):
        PI_2 = np.pi / 2
        cur_joint_data = JointData.from_joint_state(self.cur_joint_state)
        matrix = np.array(
            [
                [
                    -PI_2,
                    L12,
                    0.0,
                    cur_joint_data.joint1,
                ],
                [],
                [],
                [],
                [],
            ]
        )
