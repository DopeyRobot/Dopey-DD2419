#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState




class JointData:

    def __init__(self) -> None:
        self.joint1 = 0
        self.joint2 = 0
        self.joint3 = 0
        self.joint4 = 0
        self.joint5 = 0

    def from_joint_state(state : JointState):
        data = JointData()
        data.joint1 = state.position[0]
        data.joint2 = state.position