#!/usr/bin/env python3
import numpy as np
from kinematics_utils import JointData

# link lengths of the robot in meters
D1 = 18e-3
A2 = 100e-3
A3 = 97e-3
D5 = 135e-3
PI_2 = np.pi / 2


class KinematicsSolver:
    def __init__(self) -> None:
        self.verbose = False

    def get_DH_matrix(self, joint_data: JointData) -> np.ndarray:
        """ "
        columns are alpha, d, a, theta
        """

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
                [-PI_2, D1, 0.0, joint_data.joint1],
                [0.0, 0.0, A2, joint_data.joint2 - PI_2],
                [0.0, 0.0, A3, joint_data.joint3],
                [PI_2, 0.0, 0.0, joint_data.joint4 + PI_2],
                [0.0, D5, 0.0, joint_data.joint5],
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

    def get_DH_transform(self, joint_data: JointData, link_number: int) -> np.ndarray:
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

    def get_transform(self, joint_data: JointData, link_number: int) -> np.ndarray:
        """
        returns the transform matrix from the base to the given link number
        """
        transform = np.eye(4)
        for i in range(link_number):
            transform = np.dot(transform, self.get_DH_transform(joint_data, i))

        return transform

    def forwards_kinematics(self, joint_data: JointData) -> np.ndarray:
        """
        returns the transform matrix from the base to the end effector
        """
        transform = self.get_transform(joint_data, 5)
        R = transform[:3, :3]
        pe0 = np.array([0.0, 0.0, 0.0, 1.0]).transpose()
        pe_b = transform.dot(pe0)
        return pe_b[:3], R

    def get_end_effector_pose(self, joint_data: JointData) -> np.ndarray:
        """
        returns the transform matrix from the base to the end effector
        """
        transform = self.get_transform(joint_data, 5)
        pe0 = np.array([0.0, 0.0, 0.0, 1.0]).transpose()
        pe_b = transform.dot(pe0)
        return pe_b[:3]

    def get_link_pose(self, joint_data: JointData, link_number: int) -> np.ndarray:
        """
        returns the transform matrix from the base to the given link number
        """
        transform = self.get_transform(joint_data, link_number)
        pe0 = np.array([0.0, 0.0, 0.0, 1.0]).transpose()
        pe_b = transform.dot(pe0)

        return pe_b[:3]

    def get_link_normal(self, joint_data: JointData, link_number: int) -> np.ndarray:
        """
        returns the normal vector of the given link number
        """
        transform = self.get_transform(joint_data, link_number)
        R = transform[:3, :3]
        zi0 = np.array([0.0, 0.0, 1.0]).transpose()
        zi_b = R.dot(zi0)

        return zi_b

    def get_jacobian_column(
        self, joint_data: JointData, link_number: int
    ) -> np.ndarray:
        """
        returns the jacobian column for the given link number
        """
        zi = self.get_link_normal(joint_data, link_number)
        pi = self.get_link_pose(joint_data, link_number)
        pe = self.get_end_effector_pose(
            joint_data,
        )

        return np.concatenate([np.cross(zi, pe - pi), zi])

    def get_jacobian(self, joint_data: JointData) -> np.ndarray:
        """
        returns the jacobian matrix
        """
        jacobian = np.zeros((6, 5))
        for i in range(5):
            jacobian[:, i] = self.get_jacobian_column(joint_data, i)

        return jacobian

    def get_inverse_jacobian(self, jacobian) -> np.ndarray:
        """
        returns the inverse jacobian matrix
        """
        return np.linalg.pinv(jacobian)

    def loss(self, error: np.ndarray) -> float:
        pos_error = error[:3].dot(error[:3])
        rot_error = error[3:].dot(error[3:])
        # print(f"pos error = {pos_error}")
        # print(f"rot_error = {rot_error}")

        return pos_error + rot_error

    def get_error_vector(self, X, X_hat, R, R_hat) -> np.ndarray:
        pos_error = X_hat - X
        R_hat = R_hat.transpose()
        rot_error = -0.001 * (
            np.cross(R_hat[:, 0], R[:, 0])
            + np.cross(R_hat[:, 1], R[:, 1])
            + np.cross(R_hat[:, 2], R[:, 2])
        )

        return np.concatenate([pos_error, rot_error])

    def solve_next_position(
        self,
        desired_pos,
        desired_R,
        joint_data: JointData,
        tol=1e-12,
        lr=1,
        max_iter=10,
        verbose=True,
    ) -> JointData:
        """
        fast convergence on position, but slow on R.
        """
        R = np.array(desired_R)
        X_hat, R_hat = self.forwards_kinematics(joint_data)
        X = desired_pos
        theta = joint_data.to_np_array()
        k = 1
        _loss = self.loss(self.get_error_vector(X, X_hat, R, R_hat))
        if verbose:
            print("solving ...")
        while _loss > tol:

            jacobian = self.get_jacobian(JointData.from_np_array(theta))
            inverse_jacobian = self.get_inverse_jacobian(jacobian)
            error = self.get_error_vector(X, X_hat, R, R_hat)
            err_theta = inverse_jacobian.dot(error)
            theta = theta - lr * err_theta
            if verbose:
                print(f"loss = {_loss}")
                print(f"X = {X}, Xh = {X_hat}")
                # print(f"error = {error[:3]}")
            X_hat, R_hat = self.forwards_kinematics(JointData.from_np_array(theta))
            _loss = self.loss(error)
            k += 1
            if k > max_iter:
                if verbose:
                    print("timeout")
                print(desired_pos)
                raise RuntimeError()
        return JointData.from_np_array(theta)
