from enum import Enum
import numpy as np


class JointData:
    """
    Class to carry joint data in a more convenient way, without using a ros message
    """

    def __init__(self) -> None:
        self.joint1 = 0
        self.joint2 = 0
        self.joint3 = 0
        self.joint4 = 0
        self.joint5 = 0
        self.gripper = 0

    @staticmethod
    def from_joint_state(state):
        data = JointData()
        data.joint1 = state.position[0]
        data.joint2 = state.position[1]
        data.joint3 = state.position[2]
        data.joint4 = state.position[3]
        data.joint5 = state.position[4]
        data.gripper = state.position[5]

        return data

    @staticmethod
    def from_np_array(state: np.ndarray, include_gripper: bool = False):
        data = JointData()
        data.joint1 = state[0]
        data.joint2 = state[1]
        data.joint3 = state[2]
        data.joint4 = state[3]
        data.joint5 = state[4]
        if include_gripper:
            data.gripper = state[5]

        return data

    @staticmethod
    def from_list(positions: list):
        data = JointData()
        data.joint1 = positions[0]
        data.joint2 = positions[1]
        data.joint3 = positions[2]
        data.joint4 = positions[3]
        data.joint5 = positions[4]
        data.gripper = positions[5]

        return data

    def to_np_array(self, include_gripper: bool = False):
        if include_gripper:
            return np.array(
                [
                    self.joint1,
                    self.joint2,
                    self.joint3,
                    self.joint4,
                    self.joint5,
                    self.gripper,
                ]
            )
        return np.array(
            [
                self.joint1,
                self.joint2,
                self.joint3,
                self.joint4,
                self.joint5,
            ]
        )


class RefPoses(Enum):
    """
    Enum to carry references poses for the robot in joint space:
    - HOME: robot in home position
    - PICKUP_R: robot in position to pick up an object from the right
    - PICKUP_F: robot in position to pick up an object from the front
    - PICKUP_TRAY: robot in position to pick up an object from the tray
    - OPEN_GRIPPER: contains open gripper position
    - CLOSE_GRIPPER: contains closed gripper position
    """

    HOME = JointData.from_list(positions=[0, 0, 0, 0, 0, 0])
    PICKUP_R = JointData.from_list(positions=[0, -1, -0.8, -1.3, 0, -0.2])
    PICKUP_F = JointData.from_list(positions=[1.57, -1, -0.8, -1.3, 0, -0.2])
    PICKUP_TRAY = JointData.from_list(positions=[-1.57, 0.7, -1.57, -1.5, 0, -0.2])
    OPEN_GRIPPER = JointData.from_list(positions=[0, 0, 0, 0, 0, -1.5])
    CLOSE_GRIPPER = JointData.from_list(positions=[0, 0, 0, 0, 0, 0.3])
