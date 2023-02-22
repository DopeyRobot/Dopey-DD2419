#!/usr/bin/env python3
import rospy
from kinematics_utils import JointData, RefPoses
from kinematics_solver import KinematicsSolver, D1, A2, A3, D5
from sensor_msgs.msg import JointState
from kinematics.srv import JointAngles, IKData
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
import numpy as np

class IKService:
    """
    Service node to move the robot to a specific position in space
    """
    def __init__(self) -> None:
        self.solver = KinematicsSolver()
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        self.cur_joint_state = JointState()
        rospy.wait_for_service("pose_service", 5.0)
        self.pose_service = rospy.ServiceProxy("pose_service", JointAngles)
        self.inverse_k_service = rospy.Service("IK_service", IKData, self.IK_callback)

        while len(self.cur_joint_state.position) < 5:
            rospy.sleep(0.1)

    def IK_callback(self, req:IKData):
        pos = req.x, req.y, req.z
        psi = req.wrist_pitch
        rot = req.gripper_rotation

        self.move_to(pos, psi, rot)

        return EmptyResponse()

    def joint_state_callback(self, msg):
        self.cur_joint_state = msg

    def move_to(self, pos, wrist_pitch, wrist_rotation):
        sol = self.solver.analytical_IK(pos, wrist_pitch, wrist_rotation)
        if sol is not None:
            self.pose_service(sol.to_np_array(), 2000)
        else:
            return

if __name__ == "__main__":
    rospy.init_node("IK_service")
    IKService()
    rospy.spin()

