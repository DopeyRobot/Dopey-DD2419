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

        self.angle_increment = 0.1

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
    
    def transform_to_arm_frame(self, pose:PoseStamped) ->PoseStamped:
        transformed_pose = self.buffer.transform(pose, self.arm_frame, rospy.Duration(2))
        rospy.loginfo(transformed_pose)
        return transformed_pose


    def IK_callback(self, req:IKData):
        pose = PoseStamped()
        pose.pose.position.x = req.x
        pose.pose.position.y = req.y
        pose.pose.position.z = req.z
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "base_link"
        tranformed_pose:PoseStamped = self.transform_to_arm_frame(pose)
        pos = tranformed_pose.pose.position.x, tranformed_pose.pose.position.y, tranformed_pose.pose.position.z
        rospy.loginfo(pos)
        psi = req.wrist_pitch
        rot = req.gripper_rotation

        self.move_to(pos, psi, rot)
        if not sucess:

            for angle in self.angles_to_try:
                sucess = self.move_to(pos, angle, rot)
                if sucess:
                    break

        return EmptyResponse()

    def joint_state_callback(self, msg):
        self.cur_joint_state = msg

    def move_to(self, pos, wrist_pitch, wrist_rotation):
        sol = self.solver.analytical_IK(pos, wrist_pitch, wrist_rotation)
        if sol is not None:
            self.pose_service(sol.to_np_array(), 2000)
            rospy.loginfo("found solution")
            return True
        else:
            rospy.loginfo(f"couldn't solve pose for wrist pictch {wrist_pitch}")
            return False

if __name__ == "__main__":
    rospy.init_node("IK_service")
    IKService()
    rospy.spin()

