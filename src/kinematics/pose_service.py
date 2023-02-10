#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from enum import Enum
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from std_msgs.msg import Float64
import numpy as np
from hiwonder_servo_msgs.msg import CommandDuration


class JointData:
    def __init__(self) -> None:
        self.joint1 = 0
        self.joint2 = 0
        self.joint3 = 0
        self.joint4 = 0
        self.joint5 = 0
        self.gripper = 0

    @staticmethod
    def from_joint_state(state: JointState):
        data = JointData()
        data.joint1 = state.position[0]
        data.joint2 = state.position[1]
        data.joint3 = state.position[2]
        data.joint4 = state.position[3]
        data.joint5 = state.position[4]
        data.gripper = state.position[5]

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

    @staticmethod
    def to_np_array(data: "JointData", include_gripper: bool = False):
        if include_gripper:
            return np.array(
                [
                    data.joint1,
                    data.joint2,
                    data.joint3,
                    data.joint4,
                    data.joint5,
                    data.gripper,
                ]
            )
        return np.array(
            [
                data.joint1,
                data.joint2,
                data.joint3,
                data.joint4,
                data.joint5,
            ]
        )


class RefPoses(Enum):
    HOME = JointData.from_list(positions=[0, 0, 0, 0, 0, 0])
    PICKUP_R = JointData.from_list(positions=[0, -1, -0.8, -1.3, 0, -0.2])
    PICKUP_F = JointData.from_list(positions=[1.57, -1, -0.8, -1.3, 0, -0.2])
    PICKUP_TRAY = JointData.from_list(positions=[-1.57, 0.7, -1.57, -1.5, 0, -0.2])
    OPEN_GRIPPER = JointData.from_list(positions=[0, 0, 0, 0, 0, -1.5])
    CLOSE_GRIPPER = JointData.from_list(positions=[0, 0, 0, 0, 0, 0.3])


class PoseService:
    def __init__(self) -> None:
        rospy.init_node("pose_server")
        self.home_service = rospy.Service("home", Empty, self.home_callback)
        self.pickup_service_right = rospy.Service(
            "pickup/right", Empty, self.pickup_r_callback
        )
        self.pickup_service_front = rospy.Service(
            "pickup/front", Empty, self.pickup_f_callback
        )
        self.pickup_service_tray = rospy.Service(
            "pickup/tray", Empty, self.pickup_t_callback
        )
        self.open_gripper_service = rospy.Service(
            "gripper/open", Empty, self.open_gripper_callback
        )
        self.close_gripper_service = rospy.Service(
            "gripper/close", Empty, self.close_gripper_callback
        )

        self.joint1_pub = rospy.Publisher(
            "/joint1_controller/command_duration", CommandDuration, queue_size=10
        )
        self.joint2_pub = rospy.Publisher(
            "/joint2_controller/command_duration", CommandDuration, queue_size=10
        )
        self.joint3_pub = rospy.Publisher(
            "/joint3_controller/command_duration", CommandDuration, queue_size=10
        )
        self.joint4_pub = rospy.Publisher(
            "/joint4_controller/command_duration", CommandDuration, queue_size=10
        )
        self.joint5_pub = rospy.Publisher(
            "/joint5_controller/command_duration", CommandDuration, queue_size=10
        )
        self.gripper_pub = rospy.Publisher(
            "r_joint_controller/command_duration", CommandDuration, queue_size=10
        )

        self.joint_state_sub = rospy.Subscriber(
            "/joint_states", JointState, self.join_state_callback
        )
        self.cur_joint_state = JointState()

    def join_state_callback(self, state: JointState):
        self.cur_joint_state = state

    def home_callback(self, req: EmptyRequest) -> EmptyResponse:
        rospy.loginfo("Moving to home position")
        self.publish_data(RefPoses.HOME.value, include_gripper=True)
        return EmptyResponse()

    def pickup_r_callback(self, req: EmptyRequest) -> EmptyResponse:
        rospy.loginfo("Moving to right pickup position")
        self.publish_data(RefPoses.PICKUP_R.value)
        return EmptyResponse()

    def pickup_f_callback(self, req: EmptyRequest) -> EmptyResponse:
        rospy.loginfo("Moving to front pickup position")
        self.publish_data(RefPoses.PICKUP_F.value)
        return EmptyResponse()

    def pickup_t_callback(self, req: EmptyRequest) -> EmptyResponse:
        rospy.loginfo("Moving to tray pickup position")
        self.publish_data(RefPoses.PICKUP_TRAY.value)
        return EmptyResponse()

    def publish_data(self, joint_data: JointData, include_gripper: bool = False):
        self.joint1_pub.publish(self.to_command_duration(joint_data.joint1))
        self.joint2_pub.publish(self.to_command_duration(joint_data.joint2))
        self.joint3_pub.publish(self.to_command_duration(joint_data.joint3))
        self.joint4_pub.publish(self.to_command_duration(joint_data.joint4))
        self.joint5_pub.publish(self.to_command_duration(joint_data.joint5))

        if include_gripper:
            self.gripper_pub.publish(self.to_command_duration(joint_data.gripper))
        else:
            self.gripper_pub.publish(
                self.to_command_duration(
                    JointData.from_joint_state(self.cur_joint_state).gripper
                )
            )

    def open_gripper_callback(self, req: EmptyRequest) -> EmptyResponse:
        rospy.loginfo("Opening gripper")
        self.gripper_pub.publish(
            self.to_command_duration(RefPoses.OPEN_GRIPPER.value.gripper, duration=1000)
        )

        return EmptyResponse()

    def close_gripper_callback(self, req: EmptyRequest) -> EmptyResponse:
        rospy.loginfo("Closing gripper")
        self.gripper_pub.publish(
            self.to_command_duration(
                RefPoses.CLOSE_GRIPPER.value.gripper, duration=1000
            )
        )

        return EmptyResponse()

    @staticmethod
    def to_command_duration(position: float, duration: float = 2000) -> CommandDuration:
        command = CommandDuration()
        command.data = position
        command.duration = duration

        return command


if __name__ == "__main__":
    PoseService()
    rospy.spin()
