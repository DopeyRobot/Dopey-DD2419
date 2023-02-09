#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from enum import Enum
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from std_msgs.msg import Float64
from hiwonder_servo_msgs.msg import CommandDuration


class JointData:
    def __init__(self) -> None:
        self.joint1 = 0
        self.joint2 = 0
        self.joint3 = 0
        self.joint4 = 0
        self.joint5 = 0
        self.gripper = 0

    def from_joint_state(state: JointState):
        data = JointData()
        data.joint1 = state.position[0]
        data.joint2 = state.position[1]
        data.joint3 = state.position[2]
        data.joint4 = state.position[3]
        data.joint5 = state.position[4]
        data.gripper = state.position[5]

        return data

    def from_list(positions: list):
        data = JointData()
        data.joint1 = positions[0]
        data.joint2 = positions[1]
        data.joint3 = positions[2]
        data.joint4 = positions[3]
        data.joint5 = positions[4]
        data.gripper = positions[5]

        return data        


class RefPoses(Enum):
    HOME = JointData.from_list(positions=[0, 0, 0, 0, 0, 0])
    PICKUP_R = JointData.from_list(positions=[0, -1, -0.8, -1.3, 0, -0.2])
    PICKUP_F = JointData.from_list(positions=[1.57, -1, -0.8, -1.3, 0, -0.2])

class PosService:
    def __init__(self) -> None:
        rospy.init_node("pose_server")
        self.home_service = rospy.Service("home", Empty, self.home_callback)
        self.pickup_service = rospy.Service("pickup/r", Empty, self.pickup_r_callback )
        self.pickup_service = rospy.Service("pickup/f", Empty, self.pickup_f_callback )
        
        self.joint1_pub = rospy.Publisher(
            "/joint1_controller/command_duration", Float64, queue_size=10
        )
        self.joint2_pub = rospy.Publisher(
            "/joint2_controller/command_duration", Float64, queue_size=10
        )
        self.joint3_pub = rospy.Publisher(
            "/joint3_controller/command_duration", Float64, queue_size=10
        )
        self.joint4_pub = rospy.Publisher(
            "/joint4_controller/command_duration", Float64, queue_size=10
        )
        self.joint5_pub = rospy.Publisher(
            "/joint5_controller/command_duration", Float64, queue_size=10
        )
        self.gripper_pub = rospy.Publisher(
            "r_joint_controller/command_duration", Float64, queue_size=10
        )

    def home_callback(self, req: EmptyRequest) -> EmptyResponse:
        rospy.loginfo("Moving to home position")
        self.joint1_pub.publish(RefPoses.HOME.value.joint1)
        self.joint2_pub.publish(RefPoses.HOME.value.joint2)
        self.joint3_pub.publish(RefPoses.HOME.value.joint3)
        self.joint4_pub.publish(RefPoses.HOME.value.joint4)
        self.joint5_pub.publish(RefPoses.HOME.value.joint5)
        self.gripper_pub.publish(RefPoses.HOME.value.gripper)

        return EmptyResponse()

    def pickup_r_callback(self, req:EmptyRequest) -> EmptyResponse:
        rospy.loginfo("Moving to pickup position")
        self.joint1_pub.publish(RefPoses.PICKUP_R.value.joint1)
        self.joint2_pub.publish(RefPoses.PICKUP_R.value.joint2)
        self.joint3_pub.publish(RefPoses.PICKUP_R.value.joint3)
        self.joint4_pub.publish(RefPoses.PICKUP_R.value.joint4)
        self.joint5_pub.publish(RefPoses.PICKUP_R.value.joint5)
        self.gripper_pub.publish(RefPoses.PICKUP_R.value.gripper)

        return EmptyResponse()

    def pickup_f_callback(self, req:EmptyRequest) -> EmptyResponse:
        rospy.loginfo("Moving to pickup position")
        self.joint1_pub.publish(RefPoses.PICKUP_F.value.joint1)
        self.joint2_pub.publish(RefPoses.PICKUP_F.value.joint2)
        self.joint3_pub.publish(RefPoses.PICKUP_F.value.joint3)
        self.joint4_pub.publish(RefPoses.PICKUP_F.value.joint4)
        self.joint5_pub.publish(RefPoses.PICKUP_F.value.joint5)
        self.gripper_pub.publish(RefPoses.PICKUP_F.value.gripper)

        return EmptyResponse()




if __name__ == "__main__":
    PosService()
    rospy.spin()