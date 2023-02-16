#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, TwistStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs
import math
#from rrt import path 

class planning():

    def __init__(self):

        print('Check 1: init file entered')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()
        
        self.f = 10
        self.rate = rospy.Rate(self.f)
        self.transformedPose = PoseStamped() #init PoseStamped message type
        self.targetframe = "base_link"
        self.currentframe = "odom"
        self.timeout = rospy.Duration(2)

        self.publisher_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.publisher_twist = rospy.Publisher('motor_controller/twist', TwistStamped, queue_size=10)
        self.subscriber = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.callback) 

        self.run() #calls run function which will continuously in a loop publish newest point then subscribe as well as extract info for new point



    def callback(self, msg):
        ##INPUT MSG: POSE STAMPED
        print(f"Check 2 : callback entered")
        detectedPoseStamped = PoseStamped()
        detectedPoseStamped.header.stamp = msg.header.stamp
        detectedPoseStamped.header.frame_id = msg.header.frame_id
        detectedPoseStamped.pose = msg.pose

        detectedTransformStamped = TransformStamped()

        print("TimeStamp:", detectedPoseStamped.header.stamp)
        print("Output CanTransform ",self.tf_buffer.can_transform(self.targetframe, detectedPoseStamped.header.frame_id, detectedPoseStamped.header.stamp, self.timeout))

        if self.tf_buffer.can_transform(self.targetframe, detectedPoseStamped.header.frame_id, detectedPoseStamped.header.stamp, self.timeout):
            print(f"Check 3 : transform found")

            rospy.loginfo(f"Transform between target frame: {self.targetframe} and current frame: {self.currentframe} found")
            detectedTransformStamped = self.tf_buffer.lookup_transform(self.targetframe, self.currentframe, detectedPoseStamped.header.stamp, self.timeout)
            # tf2_geometry_msgs.do_transform_point(detectedPoseStamped, detectedTransformStamped)
            self.transformedPose = tf2_geometry_msgs.do_transform_pose(detectedPoseStamped, detectedTransformStamped)

            print(f"Check 4: pose transformed")


    def run(self):
        # Put in run file, because later when obstacles are introduced, the new position to be navigated to will be updated continuously
        # Publishing is done post transformation from map into base frame 
        while not rospy.is_shutdown():
            
            ##PUBLISH NAV GOAL COORDINATE
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = self.currentframe

            ##SECTION NEEDS TO BE REPLACED WITH PATH PLANNING FUNCTION WITH PUBLISHES ONTO MOVE_BASE_SIMPLE/GOAL
            # point = Point() 
            # point.x = 1
            # point.y = 2
            # point.z = 0

            # pose_stamped.pose.position = point
            # self.publisher_goal.publish(pose_stamped)

            ##PUBLISH TWIST SECTION
            twist = TwistStamped() # maybe import a TwistStamped instead of a Twist 
            twist.header.stamp = self.transformedPose.header.stamp # take this from imported message
            twist.header.frame_id = self.transformedPose.header.frame_id # take this from subscribed and manipulated message
            twist.twist.angular.z = 0.4 * math.atan2(self.transformedPose.pose.position.y, self.transformedPose.pose.position.x)
            twist.twist.linear.x = 0.05 * math.sqrt(self.transformedPose.pose.position.x** 2 + self.transformedPose.pose.position.y ** 2)
            self.publisher_twist.publish(twist)
            print("Check 5: published twist")
            self.rate.sleep()

            

if __name__ == "__main__":
    try:
        rospy.init_node("planning") 
        planning()      
        rospy.spin()
    except rospy.ROSInterruptException:
        pass