#!/usr/bin/env python3
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer
from kinematics_solver import KinematicsSolver
from kinematics_utils import JointData
import rospy
from sensor_msgs.msg import JointState
from std_srvs.srv import Empty, EmptyResponse, EmptyRequest

class FKpublisher:
    def __init__(self) -> None:
        self.solver = KinematicsSolver()
        self.joint_state_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_callback)
        self.cur_joint_state = JointState()
        self.cur_joint_state.position = [0,0,0,0,0,0]
        self.FK_service = rospy.Service("end_effector_pose", Empty, self.FK_callback)

    def joint_state_callback(self, msg):
        self.cur_joint_state = msg

    def FK_callback(self, req):
        pos, R = self.solver.forwards_kinematics(JointData.from_joint_state(self.cur_joint_state))
        rospy.loginfo(f"cur end effector position : \n {pos}")
        rospy.loginfo(f"cur end effector orientation : \n {R}")

        return EmptyResponse()

if __name__ == "__main__":
    rospy.init_node("forwards_kinematics")
    FKpublisher()
    rospy.spin()