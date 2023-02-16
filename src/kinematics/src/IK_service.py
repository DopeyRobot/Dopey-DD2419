#!/usr/bin/env python3
import rospy
from kinematics_utils import JointData, RefPoses
from kinematics_solver import KinematicsSolver, D1, A2, A3, D5
from sensor_msgs.msg import JointState
from kinematics.srv import JointAngles
import numpy as np

class IKService:

    def __init__(self) -> None:
        self.solver = KinematicsSolver()
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        self.cur_joint_state = JointState()
        rospy.wait_for_service("pose_service", 5.0)
        self.pose_service = rospy.ServiceProxy("pose_service", JointAngles)

        while len(self.cur_joint_state.position) < 5:
            rospy.sleep(0.1)

        sol = IKService.analytical_IK([-150e-3,0e-3,-50e-3],np.pi)
        self.pose_service.call(sol.to_np_array(), 2000)

    def joint_state_callback(self, msg):
        self.cur_joint_state = msg

    def analytical_IK(desired_pos, desired_wrist_pitch, desired_theta5 = 0.0):
        """
        Solves the analytical IK sometimes
        """
        
        try:
            psi = desired_wrist_pitch
            xd, yd, zd = desired_pos
            theta1 = np.arctan2(yd, xd)%(2*np.pi) - np.pi
            rd = np.hypot(xd, yd)
            z4 = zd - D5*np.cos(psi)
            r4 = rd - D5*np.sin(psi)

            alpha = np.arctan2(r4, z4-D1)
            S = np.hypot(z4-D1, r4)
            cb = (S**2 + A2**2 - A3**2)/(2*S*A2)
            c3 = (S*cb-A2)/(A3)

            beta= np.arccos(cb)
            theta2 = alpha - beta
            theta3 = np.arccos(c3)
            theta4 = (psi - theta2 - theta3)%(2*np.pi)
            theta5 = desired_theta5

            return JointData.from_np_array(
                -np.array([-theta1, theta2, theta3, theta4, theta5])
                )
        except:
            rospy.logerr("Unreachable pose")
            return RefPoses.HOME.value

if __name__ == "__main__":
    rospy.init_node("IK_ser")
    IKService()
    rospy.spin()

