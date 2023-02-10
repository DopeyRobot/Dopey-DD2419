#!/usr/bin/env python3
import numpy as np
import rospy
from sensor_msgs.msg import JointState
from pose_service import JointData, RefPoses
from geometry_msgs.msg import Quaternion

# link lengths of the robot in meters
L12 = 18e-3
L23 = 100e-3
L34 = 97e-3
L45 = 55e-3
LEND = 50e-3


class KinematicsSolver:
    def __init__(self) -> None:
        self.joint_state_sub = rospy.Subscriber(
            "/joint_states", JointState, self.joint_state_callback
        )
        self.cur_joint_state = JointState()
        self.verbose = False
        self.rate = rospy.Rate(1)

        self.run()

    def run(self):
        while not rospy.is_shutdown():
            if len(self.cur_joint_state.position) > 0:
                joint_data = JointData.from_joint_state(self.cur_joint_state)
                pos, R = self.forwards_kinematics(joint_data)
                rospy.loginfo(f"End effector position: {pos}")
                rospy.loginfo(f"End effector rotation matrix: {R}")
            self.rate.sleep()

    def joint_state_callback(self, msg):
        if self.verbose:
            rospy.loginfo("Received new joint state")
        self.cur_joint_state = msg

    def get_DH_matrix(self, joint_data: JointData):
        """ "
        columns are alpha, d, a, theta
        """
        PI_2 = np.pi / 2

        ## first matrix to test
        # matrix = np.array(
        #     [
        #         [-PI_2, L12, 0.0, joint_data.joint1],
        #         [0.0, 0.0, L23, joint_data.joint2 - PI_2],
        #         [0.0, 0.0, L34, joint_data.joint3],
        #         [0.0, 0.0, L45, joint_data.joint4],
        #         [joint_data.joint5, 0.0, LEND, 0.0],
        #     ]
        # )

        # second matrix to test (seems to be the right one :) )
        matrix = np.array(
            [
                [-PI_2, L12, 0.0, joint_data.joint1],
                [0.0, 0.0, L23, joint_data.joint2 - PI_2],
                [0.0, 0.0, L34, joint_data.joint3],
                [PI_2, 0.0, 0.0, joint_data.joint4 + PI_2],
                [0.0, L45 + LEND, 0.0, joint_data.joint5],
            ]
        )

        ## third matrix to test
        # matrix = np.array(
        #     [
        #         [-PI_2, L12, 0.0, joint_data.joint1],
        #         [0.0, 0.0, L23, joint_data.joint2],
        #         [0.0, 0.0, L34, joint_data.joint3],
        #         [PI_2, 0.0, 0.0, joint_data.joint4],
        #         [0.0, L45 + LEND, 0.0, joint_data.joint5],
        #     ]
        # )

        return matrix

    def get_DH_transform(self, joint_data: JointData, link_number: int):
        """
        returns the transform matrix for the given link number
        """
        alpha, d, a, theta = self.get_DH_matrix(joint_data)[link_number, :]
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

    def get_transform(self, joint_data: JointData, link_number: int):
        """
        returns the transform matrix from the base to the given link number
        """
        transform = np.eye(4)
        for i in range(link_number):
            transform = np.dot(transform, self.get_DH_transform(joint_data, i))

        return transform

    def forwards_kinematics(self, joint_data: JointData):
        """
        returns the transform matrix from the base to the end effector
        """
        transform = self.get_transform(joint_data, 5)
        R = transform[:3, :3]
        pe0 = np.array([0.0, 0.0, 0.0, 1.0]).transpose()
        pe_b = transform.dot(pe0)
        return pe_b[:3], R

    def get_end_effector_pose(self, joint_data: JointData):
        """
        returns the transform matrix from the base to the end effector
        """
        transform = self.get_transform(joint_data, 5)
        pe0 = np.array([0.0, 0.0, 0.0, 1.0]).transpose()
        pe_b = transform.dot(pe0)
        return pe_b[:3]

    def get_link_pose(self, joint_data: JointData, link_number: int):
        """
        returns the transform matrix from the base to the given link number
        """
        transform = self.get_transform(joint_data, link_number)
        pe0 = np.array([0.0, 0.0, 0.0, 1.0]).transpose()
        pe_b = transform.dot(pe0)

        return pe_b[:3]

    def get_link_normal(self, joint_data: JointData, link_number: int):
        """
        returns the normal vector of the given link number
        """
        transform = self.get_transform(joint_data, link_number)
        R = transform[:3, :3]
        zi0 = np.array([0.0, 0.0, 1.0]).transpose()
        zi_b = R.dot(zi0)

        return zi_b

    def get_jacobian_column(self, joint_data: JointData, link_number: int):
        """
        returns the jacobian column for the given link number
        """
        zi = self.get_link_normal(joint_data, link_number)
        pi = self.get_link_pose(joint_data, link_number)
        pe = self.get_end_effector_pose(
            joint_data,
        )

        return np.concatenate([np.cross(zi, pe - pi), zi])

    def get_jacobian(self, joint_data: JointData):
        """
        returns the jacobian matrix
        """
        jacobian = np.zeros((6, 5))
        for i in range(5):
            jacobian[:, i] = self.get_jacobian_column(joint_data, i)

        return jacobian

    def get_inverse_jacobian(self, jacobian):
        """
        returns the inverse jacobian matrix
        """
        return np.linalg.pinv(jacobian)

    def loss(self, error: np.ndarray):

        return error.dot(error)

    def get_error_vector(self, X, X_hat, R, R_hat):
        pos_error = X_hat - X
        R_hat = R_hat.transpose()
        rot_error = -0.5 * (
            np.cross(R_hat[:, 0], R[:, 0])
            + np.cross(R_hat[:, 1], R[:, 1])
            + np.cross(R_hat[:, 2], R[:, 2])
        )

        return np.concatenate([pos_error, rot_error])

    def solve_next_position(
        self,
        desired_pos,
        desired_R,
        joint_state: JointData,
        tol=1e-4,
        lr=1,
        verbose=False,
    ):
        """
        fast convergence on position, but slow on R.
        """
        R = np.array(desired_R)
        X_hat, R_hat = self.forwards_kinematics()
        X = desired_pos
        theta = joint_state.to_np_array()
        k = 1
        _loss = self.loss(self.get_error_vector(X, X_hat, R, R_hat))
        if verbose:
            print("solving ...")
        while _loss > tol:
            jacobian = self.get_jacobian()
            inverse_jacobian = self.get_inverse_jacobian(jacobian)
            error = self.get_error_vector(X, X_hat, R, R_hat)
            err_theta = inverse_jacobian.dot(error)
            theta = theta - lr * err_theta
            X_hat, R_hat = self.forwards_kinematics(JointData.from_np_array(theta))
            _loss = self.loss(error)
            k += 1
            lr /= 10
            if k > 7:
                if verbose:
                    print("timeout")
                raise RuntimeError()
        return theta

    def ros_quaternion_to_rotation(self, quaternion: Quaternion):
        return np.array(
            [
                [
                    1 - 2 * (quaternion.y**2 + quaternion.z**2),
                    2 * (quaternion.x * quaternion.y - quaternion.z * quaternion.w),
                    2 * (quaternion.x * quaternion.z + quaternion.y * quaternion.w),
                ],
                [
                    2 * (quaternion.x * quaternion.y + quaternion.z * quaternion.w),
                    1 - 2 * (quaternion.x**2 + quaternion.z**2),
                    2 * (quaternion.y * quaternion.z - quaternion.x * quaternion.w),
                ],
                [
                    2 * (quaternion.x * quaternion.z - quaternion.y * quaternion.w),
                    2 * (quaternion.y * quaternion.z + quaternion.x * quaternion.w),
                    1 - 2 * (quaternion.x**2 + quaternion.y**2),
                ],
            ]
        )


if __name__ == "__main__":
    rospy.init_node("kinematics_solver")
    solver = KinematicsSolver()
    rospy.spin()
