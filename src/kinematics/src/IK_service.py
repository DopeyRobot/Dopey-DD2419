#!/usr/bin/env python3
import rospy
from kinematics_utils import JointData, RefPoses
from kinematics_solver import KinematicsSolver, D1, A2, A3, D5
from sensor_msgs.msg import JointState
from kinematics.srv import JointAngles, IKData, JointAnglesRequest
from std_srvs.srv import (
    Empty,
    EmptyResponse,
    EmptyRequest,
    Trigger,
    TriggerRequest,
    TriggerResponse,
)
from object_detection.srv import XnY, XnYRequest, XnYResponse
from std_msgs.msg import String
from tf2_geometry_msgs import PoseStamped
from tf2_ros import Buffer, TransformListener
from kinematics.srv import GripStrength, GripStrengthRequest
import numpy as np


class IKService:
    """
    Service node to move the robot to a specific position in space
    """

    def __init__(self) -> None:
        self.solver = KinematicsSolver()
        self.joint_state_sub = rospy.Subscriber(
            "/joint_states", JointState, self.joint_state_callback
        )
        self.cur_joint_state = JointState()
        self.buffer = Buffer(rospy.Duration(2))
        self.listener = TransformListener(self.buffer)
        self.arm_frame = "arm_base_link"
        rospy.wait_for_service("pose_service", 5.0)
        self.pose_service = rospy.ServiceProxy("pose_service", JointAngles)
        self.gripper_open_service = rospy.ServiceProxy("gripper/open", Trigger)
        self.gripper_close_service = rospy.ServiceProxy("gripper/close", GripStrength)
        self.blob_service = rospy.ServiceProxy("blob_detection", XnY)
        self.pickup_goal = PoseStamped()
        self.inverse_k_service = rospy.Service("IK_service", IKData, self.IK_callback)
        self.pickup_goal_sub = rospy.Subscriber(
            "pickup_goal", PoseStamped, self.pickup_goal_callback
        )
        self.pickup_service = rospy.Service("pickup_pose", Empty, self.pickup_callback)
        self.full_pickup_service = rospy.Service(
            "full_pickup_pose", Trigger, self.pickup_routine_callback
        )

        self.angle_increment = 0.1
        self.rot = 0

        while len(self.cur_joint_state.position) < 5:
            rospy.sleep(0.1)

    def pickup_goal_callback(self, msg):
        msg.header.frame_id = "base_link"
        self.pickup_goal = msg

    def pickup_callback(self, req: Empty):
        """
        goes to the pickup goal
        """
        x, y, z = (
            self.pickup_goal.pose.position.x,
            self.pickup_goal.pose.position.y,
            self.pickup_goal.pose.position.z,
        )

        if x < 0.05:
            x = 0.05
        if y > 0.10:
            y = 0.10
        if y < -0.10:
            y = -0.10

        z = -0.06

        psi = -np.pi
        rot = self.rot

        req = IKData()
        req.x = x
        req.y = y
        req.z = z
        req.wrist_pitch = psi
        req.gripper_rotation = rot

        self.IK_callback(req)

        return EmptyResponse()

    def transform_to_arm_frame(self, pose: PoseStamped) -> PoseStamped:
        transformed_pose = self.buffer.transform(
            pose, self.arm_frame, rospy.Duration(2)
        )
        return transformed_pose

    def IK_callback(self, req: IKData):
        pose = PoseStamped()
        pose.pose.position.x = req.x
        pose.pose.position.y = req.y
        pose.pose.position.z = req.z
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_link"
        transformed_pose: PoseStamped = self.transform_to_arm_frame(pose)
        pos = (
            transformed_pose.pose.position.x,
            transformed_pose.pose.position.y,
            transformed_pose.pose.position.z,
        )
        rospy.loginfo(pos)
        psi = req.wrist_pitch
        rot = req.gripper_rotation

        sucess = self.move_to(pos, psi, rot)
        if not sucess:
            rospy.loginfo("trying a new angle")
            for i in range(1, 10):
                sucess = self.move_to(pos, psi - self.angle_increment * i, rot)
                if sucess:
                    break

        return EmptyResponse()

    def joint_state_callback(self, msg):
        self.cur_joint_state = msg

    def move_to(self, pos, wrist_pitch, wrist_rotation):
        sol = self.solver.analytical_IK(pos, wrist_pitch, wrist_rotation, reflect_angle=False)
        if sol is not None:
            rospy.loginfo(sol)
            self.pose_service(sol.to_np_array(), 800)
            rospy.loginfo("found solution")
            return True
        else:
            rospy.loginfo(f"couldn't solve pose for wrist pictch {wrist_pitch}")
            return False

    def pickup_routine_callback(self, req: TriggerRequest) -> TriggerResponse:
        rospy.loginfo("pickup routine started")
        self.pose_service(RefPoses.PREPICK_F.value.to_joint_angles_req())
        rospy.sleep(3)
        rospy.loginfo("detecting object ....")
        xnyreq = XnYRequest()
        xnyresp = self.blob_service(xnyreq)
        angle = xnyresp.h
        print(angle)
        angle= np.deg2rad(angle)
        self.rot=angle
        rospy.sleep(2)
        self.gripper_open_service(TriggerRequest())
        rospy.sleep(2)
        rospy.loginfo("moving to pick")
        self.pickup_callback(EmptyRequest())
        rospy.sleep(2)
        rospy.loginfo("closing gripper")
        grip_req = GripStrengthRequest(String("cube"))
        self.gripper_close_service(grip_req)
        rospy.sleep(2)
        rospy.loginfo("going back home")
        self.pose_service(RefPoses.HOME.value.to_joint_angles_req())
        rospy.sleep(2)
        return TriggerResponse(True, "pickup routine finished")


if __name__ == "__main__":
    rospy.init_node("IK_service")
    IKService()
    rospy.spin()
