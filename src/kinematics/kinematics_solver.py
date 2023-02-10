#!/usr/bin/env python3
import numpy as np
import rospy
import tf2_ros as tf2
from sensor_msgs.msg import JointState
from pose_service import JointData

# link lengths of the robot in meters
L12 = 18e-3
L23 = 100e-3
L34 = 97e-3
L45 = 55e-3
LEND = 50e-3


class KinematicsSolver:
    def __init__(self) -> None:
        self.joint_state_sub = rospy.Subscriber(
            "/joint_states", JointState, self.join_state_callback
        )
        self.cur_joint_state = JointState()
        self.verbose = False
        self.rate = rospy.Rate(1)

        self.run()

    def run(self):
        while not rospy.is_shutdown():
            pos, R = self.forwards_kinematics()
            rospy.loginfo(f"End effector position: {pos}")
            rospy.loginfo(f"End effector rotation matrix: {R}")
            self.rate.sleep()

    def joint_state_callback(self, msg):
        if self.verbose:
            rospy.loginfo("Received new joint state")
        self.cur_joint_state = msg

    def get_DH_matrix(self):
        """ "
        columns are alpha, d, a, theta
        """
        PI_2 = np.pi / 2
        cur_joint_data = JointData.from_joint_state(self.cur_joint_state)
        matrix = np.array(
            [
                [-PI_2, L12, 0.0, cur_joint_data.joint1],
                [0.0, 0.0, L23, cur_joint_data.joint2 - PI_2],
                [0.0, 0.0, L34, cur_joint_data.joint3],
                [0.0, 0.0, L45, cur_joint_data.joint4],
                [cur_joint_data.joint5, 0.0, LEND, 0.0],
            ]
        )

        return matrix

    def get_DH_transform(self, link_number: int):
        """
        returns the transform matrix for the given link number
        """
        alpha, d, a, theta = self.get_DH_matrix()[link_number, :]
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        ct = np.cos(theta)
        st = np.sin(theta)

        return np.array(
            [
                [ct, -st * ca, st * sa, a * ct],
                [st, ct * ca, -ct * sa, a * st],
                [0.0, sa, ca, d],
                [0.0, 0.0, 0.0, 1.0],
            ]
        )

    def get_transform(self, link_number: int):
        """
        returns the transform matrix from the base to the given link number
        """
        transform = np.eye(4)
        for i in range(link_number + 1):
            transform = np.dot(transform, self.get_DH_transform(i))

        return transform

    def forwards_kinematics(self):
        """
        returns the transform matrix from the base to the end effector
        """
        transform = self.get_transform(5)
        R = transform[:3, :3]
        pe0 = np.array([0.0, 0.0, 0.0, 1.0]).transpose()
        pe_b = transform.dot(pe0)
        return pe_b[:3], R

    def get_end_effector_pose(self):
        """
        returns the transform matrix from the base to the end effector
        """
        transform = self.get_transform(5)
        pe0 = np.array([0.0, 0.0, 0.0, 1.0]).transpose()
        pe_b = transform.dot(pe0)
        return pe_b[:3]

    def get_link_pose(self, link_number: int):
        """
        returns the transform matrix from the base to the given link number
        """
        transform = self.get_transform(link_number)
        pe0 = np.array([0.0, 0.0, 0.0, 1.0]).transpose()
        pe_b = transform.dot(pe0)

        return pe_b[:3]

    def get_link_normal(self, link_number: int):
        """
        returns the normal vector of the given link number
        """
        transform = self.get_transform(link_number)
        R = transform[:3, :3]
        zi0 = np.array([0.0, 0.0, 1.0]).transpose()
        zi_b = R.dot(zi0)

        return zi_b

    def get_jacobian_column(self, link_number: int):
        """
        returns the jacobian column for the given link number
        """
        zi = self.get_link_normal(link_number)
        pi = self.get_link_pose(link_number)
        pe = self.get_end_effector_pose()

        return np.concatenate([np.cross(zi, pe - pi), zi])

    def get_jacobian(self):
        """
        returns the jacobian matrix
        """
        jacobian = np.zeros((6, 5))
        for i in range(5):
            jacobian[:, i] = self.get_jacobian_column(i)

        return jacobian

    def get_inverse_jacobian(self, jacobian=None):
        """
        returns the inverse jacobian matrix
        """
        if jacobian is None:
            jacobian = self.get_jacobian()
        return np.linalg.pinv(jacobian)


if __name__ == "__main__":
    rospy.init_node("kinematics_solver")
    solver = KinematicsSolver()
    rospy.spin()
