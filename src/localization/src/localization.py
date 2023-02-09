#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from aruco_msgs.msg import MarkerArray
import tf_conversions
import tf2_ros
import math
from odometryLoc import Odometry


class Localization:
    def __init__(self) -> None:
        self.sub_anchor = rospy.Subscriber(
            "/aruco/markers", MarkerArray, self.anchor_callback
        )
        self.odom = Odometry()
        self.rate = rospy.Rate(self.odom.f)

        self.anchor = None
        self.anchorID = 2
        self.br = tf2_ros.TransformBroadcaster()

        self.run()

    def anchor_callback(self, msg):
        #rospy.loginfo("anchor callback")
        for marker in msg.markers:
            if marker.id == self.anchorID:
                self.anchor = marker
        #todo - continue here tomorrow
        if self.anchor:
            t = TransformStamped()
            t.header.frame_id = "map"
            t.child_frame_id = "odom"

            t.transform.translation.x = self.anchor.pose.pose.position.x
            t.transform.translation.y = self.anchor.pose.pose.position.y
            t.transform.translation.z = self.anchor.pose.pose.position.z

            t.transform.rotation.x = self.anchor.pose.pose.orientation.x
            t.transform.rotation.y = self.anchor.pose.pose.orientation.y
            t.transform.rotation.z = self.anchor.pose.pose.pose.orientation.z
            t.transform.rotation.w = self.anchor.pose.pose.pose.orientation.w

            self.br.sendTransform(t)
        #todo continue above tomorrow
    def predict(self):
        self.odom.step()
    
    def update(self):
        pass

    
    def run(self):
        while not rospy.is_shutdown():
            self.predict()
            self.update()
            self.rate.sleep()
        

if __name__ == "__main__":
    rospy.init_node("localization")
    Localization()
    rospy.spin()
