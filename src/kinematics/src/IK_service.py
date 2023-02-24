#!/usr/bin/env python3
import rospy
from kinematics_utils import JointData, RefPoses
from kinematics_solver import KinematicsSolver, D1, A2, A3, D5
from sensor_msgs.msg import JointState
from kinematics.srv import JointAngles, IKData
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest
from tf2_geometry_msgs import PoseStamped
from tf2_ros import Buffer, TransformListener
import numpy as np

class IKService:
    """
    Service node to move the robot to a specific position in space
    """
    def __init__(self) -> None:
        self.solver = KinematicsSolver()
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        self.cur_joint_state = JointState()
        self.buffer = Buffer(rospy.Duration(2))
        self.listener = TransformListener(self.buffer)
        self.arm_frame = "arm_base_link"
        rospy.wait_for_service("pose_service", 5.0)
        self.pose_service = rospy.ServiceProxy("pose_service", JointAngles)
        self.pickup_goal = PoseStamped()
        self.inverse_k_service = rospy.Service("IK_service", IKData, self.IK_callback)
        self.pickup_goal_sub = rospy.Subscriber("pickup_goal", PoseStamped, self.pickup_goal_callback)
        self.pickup_service = rospy.Service("pickup_pose", Empty, self.pickup_callback)

        while len(self.cur_joint_state.position) < 5:
            rospy.sleep(0.1)

    def pickup_goal_callback(self, msg):
        msg.header.frame_id = "base_link"
        self.pickup_goal = self.buffer.transform(msg, self.arm_frame, rospy.Duration(2))

    def pickup_callback(self, req:Empty):
        """
        goes to the pickup goal
        """
        x,y,z = self.pickup_goal.pose.position.x, self.pickup_goal.pose.position.y, self.pickup_goal.pose.position.z
        psi = np.pi
        rot = 0

        req = IKData()
        req.x = x
        req.y = y
        req.z = z
        req.wrist_pitch = psi
        req.gripper_rotation = rot

        self.IK_callback(req)

        return EmptyResponse()

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
            rospy.loginfo("couldn't solve pose")
            return

if __name__ == "__main__":
    rospy.init_node("IK_service")
    IKService()
    rospy.spin()

