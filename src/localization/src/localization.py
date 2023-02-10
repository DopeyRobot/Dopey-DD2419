#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, PoseStamped
from aruco_msgs.msg import MarkerArray
import tf_conversions
import tf2_ros
import math
import tf2_geometry_msgs
from odometryLoc import Odometry


class Localization:
    def __init__(self) -> None:
        self.sub_anchor = rospy.Subscriber(
            "/aruco/markers", MarkerArray, self.anchor_callback
        )

        self.odom = Odometry()
        self.rate = rospy.Rate(self.odom.f)

        #Anchor stuff
        self.anchor = None #In aruco_frame TF
        self.anchorID = 2
        
        #TF stuff
        self.aruco_frame = "camera_color_optical_frame"
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.br = tf2_ros.TransformBroadcaster()

        self.run()

    def anchor_callback(self, msg):
        #rospy.loginfo("anchor callback")
        for marker in msg.markers:
            if marker.id == self.anchorID:
                # anchor = PoseWithCovarianceStamped()
                # anchor.pose.pose = marker.pose.pose #.pose & .covariance exist after this
                # anchor.pose.covariance = marker.pose.covariance
                # anchor.header.frame_id = self.aruco_frame
                # anchor.header.stamp = msg.header.stamp

                anchor = PoseStamped()
                anchor.pose = marker.pose.pose
                anchor.header.frame_id = self.aruco_frame
                anchor.header.stamp = msg.header.stamp

                self.anchor = anchor
                
        self.place_anchor()
        
    def predict(self):
        self.odom.step()
    
    def place_anchor(self):
        if self.anchor:
            transform_is_possible = self.buffer.can_transform("odom", self.aruco_frame, self.anchor.header.stamp, rospy.Duration(2))
            rospy.loginfo(f"Tranfsorm odom aruco_frame possible: {transform_is_possible}")
            #anchorPoseStamped = self._PoseWithCovarianceStamped_to_PoseStamped(self.anchor)
            anchorPoseStamped = self.anchor
            if transform_is_possible:
                transform = self.buffer.lookup_transform("odom", self.aruco_frame, self.anchor.header.stamp, rospy.Duration(2))
                #anchor_odom_pose = self.buffer.do_transform_pose(anchorPoseStamped, "odom", rospy.Duration(2))
                anchor_odom_pose = tf2_geometry_msgs.do_transform_pose(anchorPoseStamped, transform)
                t = TransformStamped()
                t.header.frame_id = "map"
                t.child_frame_id = "odom"

                t.transform.translation.x = anchor_odom_pose.pose.position.x
                t.transform.translation.y = anchor_odom_pose.pose.position.y
                t.transform.translation.z = anchor_odom_pose.pose.position.z

                t.transform.rotation.x = anchor_odom_pose.pose.orientation.x
                t.transform.rotation.y = anchor_odom_pose.pose.orientation.y
                t.transform.rotation.z = anchor_odom_pose.pose.orientation.z
                t.transform.rotation.w = anchor_odom_pose.pose.orientation.w

            self.br.sendTransform(t)

    def _PoseWithCovarianceStamped_to_PoseStamped(self, msg):
        pose = PoseStamped()
        pose.pose = msg.pose.pose
        pose.header = msg.header
        return pose
    
    def run(self):
        while not rospy.is_shutdown():
            self.predict()
            #self.update()
            self.rate.sleep()
        

if __name__ == "__main__":
    rospy.init_node("localization")
    Localization()
    rospy.spin()
