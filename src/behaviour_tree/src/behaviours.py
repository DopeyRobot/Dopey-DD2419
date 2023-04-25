#!/usr/bin/env python3
import rospy
import tf2_geometry_msgs
import py_trees as pt
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
from nav_msgs.msg import Path
from kinematics.srv import GripStrength, GripStrengthRequest, GripStrengthResponse
import py_trees as pt


class give_path(pt.behaviour.Behaviour):
    def __init__(self):
        self.name = "give_path"
        self.goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=10)
        self.path_sub = rospy.Subscriber("/path_topic", Path, self.path_callback)
        self.ready_for_pose_sub = rospy.Subscriber(
            "/ready_for_pose", Bool, self.ready_for_path_callback
        )
        self.ready_for_pose_pub = rospy.Publisher(
            "/ready_for_pose", Bool, queue_size=1, latch=True
        )
        self.ready_for_new_path = rospy.Publisher(
            "/ready_for_new_path", Bool, queue_size=1, latch=True
        )

        self.path = None  # Path()
        self.ready_for_pose = Bool()
        self.ready_for_path = Bool()
        self.pose_to_send = 0
        # become a behaviour
        super(give_path, self).__init__("Give path!")
        self.update()

    def path_callback(self, msg):
        self.path = msg

    def ready_for_path_callback(self, msg):
        self.ready_for_pose = msg.data

    def get_current_pose(self):

        robot_pose = PoseStamped()
        robot_pose.header.stamp = rospy.Time.now()
        base_link_origin = PoseStamped()
        base_link_origin.header.stamp = robot_pose.header.stamp

        transform_to_map = self.buffer.lookup_transform(
            "base_link", "map", robot_pose.header.stamp, rospy.Duration(1)
        )  # TODO: check order of target/source frrame
        baseInMapPose = tf2_geometry_msgs.do_transform_pose(
            base_link_origin, transform_to_map
        )

        robot_pose.pose.position.z = baseInMapPose.pose.position.z
        robot_pose.pose.position.x = baseInMapPose.pose.position.x
        robot_pose.pose.position.y = baseInMapPose.pose.position.y
        robot_pose.pose.orientation.w = baseInMapPose.pose.orientation.w
        robot_pose.pose.orientation.x = baseInMapPose.pose.orientation.x
        robot_pose.pose.orientation.y = baseInMapPose.pose.orientation.y
        robot_pose.pose.orientation.z = baseInMapPose.pose.orientation.z

        robot_pose.header.frame_id = "map"

        return robot_pose

    def update(self):
        if self.path is not None:
            if self.ready_for_pose:  # and self.path.poses != []:
                # print('Ready for new pose! Sending RUNNING in tree')
                self.ready_for_pose = False
                self.ready_for_pose_pub.publish(self.ready_for_pose)
                self.goal_pub.publish(self.path.poses[self.pose_to_send])
                self.pose_to_send = self.pose_to_send + 1

                # Send RUNNING to behaviour tree
                return pt.common.Status.RUNNING

            if self.pose_to_send == len(self.path.poses):

                # Kalla på function som kollar vart man är och om det är rätt skicka success.

                self.pose_to_send = 0
                self.ready_for_path = True
                self.ready_for_new_path.publish(self.ready_for_path)

                # Should probably implement a comparison between where base_link is in tf_tree and last desired pose
                # If base_link is close enough to last desired pose, send SUCCESS to behaviour tree
                # Send SUCCESS to behaviour tree
                print("Reached final pose, sending SUCCESS in tree")
                self.__init__()  # path = None
                return pt.common.Status.SUCCESS

                # Else send FAILURE to behaviour tree
                # return pt.common.Status.FAILURE

            else:
                self.ready_for_path = False
                self.ready_for_new_path.publish(self.ready_for_path)
                # print('Moving to next pose in path array! Sending RUNNING in tree')
                return pt.common.Status.RUNNING
        elif self.path is None:
            # print("Waiting for path, none given yet! Sending RUNNING in tree")
            return pt.common.Status.RUNNING


class MoveArmToPickupFront(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__("Moving arm to pickup front")
        self.move_srv = rospy.ServiceProxy("/pickup/front", Trigger)
        rospy.wait_for_service("/pickup/front", timeout=5)
        self.tried = False
        self.done = False
        self.resp = None

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        if not self.tried:
            self.tried = True
            self.resp: TriggerResponse = self.move_srv(TriggerRequest())
            return pt.common.Status.RUNNING
        if self.resp.success:
            self.done = True
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


class MoveArmToHome(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__("Moving arm to home")
        self.move_srv = rospy.ServiceProxy("/home", Trigger)
        rospy.wait_for_service("/home", timeout=5)
        self.tried = False
        self.done = False
        self.resp = None

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        if not self.tried:
            self.tried = True
            self.resp: TriggerResponse = self.move_srv(TriggerRequest())
            return pt.common.Status.RUNNING
        if self.resp.success:
            self.done = True
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class MoveArmToUnfold(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__("Moving arm to home")
        self.move_srv = rospy.ServiceProxy("/unfold", Trigger)
        rospy.wait_for_service("/unfold", timeout=5)
        self.tried = False
        self.done = False
        self.resp = None

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        if not self.tried:
            self.tried = True
            self.resp: TriggerResponse = self.move_srv(TriggerRequest())
            return pt.common.Status.RUNNING
        if self.resp.success:
            self.done = True
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class MoveArmToDrop(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__("Moving arm to drop pos")
        self.move_srv = rospy.ServiceProxy("/drop", Trigger)
        rospy.wait_for_service("/drop", timeout=5)
        self.tried = False
        self.done = False
        self.resp = None

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        if not self.tried:
            self.tried = True
            self.resp: TriggerResponse = self.move_srv(TriggerRequest())
            return pt.common.Status.RUNNING
        if self.resp.success:
            self.done = True
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE
        
class MoveArmToTray(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__("Moving arm to tray")
        self.move_srv = rospy.ServiceProxy("/pickup/tray", Trigger)
        rospy.wait_for_service("/pickup/tray", timeout=5)
        self.tried = False
        self.done = False
        self.resp = None

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        if not self.tried:
            self.tried = True
            self.resp: TriggerResponse = self.move_srv(TriggerRequest())
            return pt.common.Status.RUNNING
        if self.resp.success:
            self.done = True
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


class OpenGripper(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__("Opening gripper")
        self.move_srv = rospy.ServiceProxy("/gripper/open", Trigger)
        rospy.wait_for_service("/gripper/open", timeout=5)
        self.tried = False
        self.done = False
        self.resp = None

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        if not self.tried:
            self.tried = True
            self.resp: TriggerResponse = self.move_srv(TriggerRequest())
            return pt.common.Status.RUNNING
        if self.resp.success:
            self.done = True
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


class CloseGripper(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__("Closing gripper")
        self.move_srv = rospy.ServiceProxy("/gripper/close", GripStrength)
        rospy.wait_for_service("/gripper/close", timeout=5)
        self.tried = False
        self.done = False
        self.resp = None

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        if not self.tried:
            self.tried = True
            req = GripStrengthRequest(String("cube"))
            self.resp: TriggerResponse = self.move_srv(req)
            return pt.common.Status.RUNNING
        if self.resp.success:
            self.done = True
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


class PickupToTarget(pt.behaviour.Behaviour):
    """
    pickup to target that is a poststamped publish to the pickup_goal topic in base_link frame
    """

    def __init__(self):
        super().__init__("Pickup to goal")
        self.move_srv = rospy.ServiceProxy("/full_pickup_pose", Trigger)
        rospy.wait_for_service("/full_pickup_pose", timeout=5)
        self.tried = False
        self.done = False
        self.resp = None

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        if not self.tried:
            self.tried = True
            self.resp: TriggerResponse = self.move_srv(TriggerRequest())
            return pt.common.Status.RUNNING
        if self.resp.success:
            self.done = True
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

class Wait(pt.behaviour.Behaviour):
    def __init__(self, duration:int):
        super().__init__("wait")
        self.duration = duration
        self.done = False
    
    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        rospy.sleep(self.duration)
        self.done = True
        return pt.common.Status.SUCCESS