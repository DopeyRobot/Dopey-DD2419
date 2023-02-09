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
        self.run()

    def anchor_callback(self, msg):
        rospy.loginfo("anchor callback")
        

    def predict(self):
        self.odom.run()

    
    def run(self):
        while not rospy.is_shutdown():
            self.predict()

            self.rate.sleep()
        


if __name__ == "__main__":
    rospy.init_node("localization")
    Localization()
    rospy.spin()
