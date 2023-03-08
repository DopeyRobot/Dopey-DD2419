#!/usr/bin/env python3
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer, TransformStamped
from kinematics_solver import KinematicsSolver
from kinematics_utils import JointData, rotation_to_ros_quaternion
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
        self.buffer = Buffer(rospy.Duration(1200))
        self.listener = TransformListener(self.buffer)
        self.broadcaster = TransformBroadcaster()
        self.f = 10
        self.rate = rospy.Rate(self.f)
        self.run()

    def joint_state_callback(self, msg):
        self.cur_joint_state = msg

    def FK_callback(self, req):
        transform: TransformStamped= self.buffer.lookup_transform("base_link", "end_effector_frame", rospy.Time(0))
        pos = transform.transform.translation.x,transform.transform.translation.y,transform.transform.translation.z
        rospy.loginfo(f"cur end effector position in base_link frame: \n {pos}")
        pos, _ = self.solver.forwards_kinematics(JointData.from_joint_state(self.cur_joint_state))
        rospy.loginfo(f"cur end effector position in arm frame: \n {pos}")


        return EmptyResponse()

    def run(self):
        while not rospy.is_shutdown():
            pos, R = self.solver.forwards_kinematics(JointData.from_joint_state(self.cur_joint_state))
            t = TransformStamped()
            t.child_frame_id = "end_effector_frame"
            t.header.frame_id = "arm_base_link"
            t.header.stamp = rospy.Time.now()
            t.transform.translation.x = pos[0]
            t.transform.translation.y = pos[1]
            t.transform.translation.z = pos[2]
            q = rotation_to_ros_quaternion(R)
            t.transform.rotation.w = q[3]
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]

            self.broadcaster.sendTransform(t)
            
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("forwards_kinematics")
    FKpublisher()
    rospy.spin()