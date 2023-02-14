#!/usr/bin/env python3
import rospy
from kinematics_utils import JointData, Trajectory
from kinematics_solver import KinematicsSolver, D1, A2, A3, D5
from sensor_msgs.msg import JointState
from kinematics.srv import JointAngles
import numpy as np

class IKService:

    def __init__(self) -> None:
        self.solver = KinematicsSolver()
        # self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        self.cur_joint_state = JointState()
        rospy.wait_for_service("pose_service", 5.0)
        # self.pose_service = rospy.ServiceProxy("pose_service", JointAngles)

        # while len(self.cur_joint_state.position) < 5:
        #     rospy.sleep(0.1)

        sol1, sol2 = IKService.analytical_IK([0,0,0.32], -np.pi/2)
        print(sol1)
        print(sol2)
        # self.pose_service.call(sol1.to_np_array(), 2000)

        pos1 = self.solver.forwards_kinematics(sol1)
        pos2 = self.solver.forwards_kinematics(sol2)
        
        print(pos1)
        print(pos2)

    def joint_state_callback(self, msg):
        self.cur_joint_state = msg

    def analytical_IK(desired_pos, desired_wrist_pitch, desired_theta5 = 0.0):
        xd = desired_pos[0]
        yd = desired_pos[1]
        zd = desired_pos[2]
        psi = desired_wrist_pitch

        theta1 = np.arctan2(yd, xd)
        d = np.hypot(xd, yd)
        r4 = d - D5*np.cos(psi)
        z4 = zd - D5*np.sin(psi)
        s = np.hypot(z4-D1, r4)
        beta = np.arctan2(s**2 + A2**2 - A3**2, 2*s*A2)
        alpha = np.arctan2(z4-D1, r4)

        theta2_1 = alpha + beta - np.pi/2
        theta2_2 = alpha - beta - np.pi/2




        theta3 = np.arctan2(s**2 - A2**2 - A3**2, 2*A2*A3)
        theta4_1 = psi - theta2_1 - theta3
        theta4_2 = psi - theta2_2 - theta3 

        return JointData.from_np_array(
            np.array([theta1, theta2_1 ,theta3, theta4_1, desired_theta5])
        ),JointData.from_np_array(
            np.array([theta1, theta2_2, theta3, theta4_2, desired_theta5])
        )
if __name__ == "__main__":
    IKService()

