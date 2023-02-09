#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import math
from odometryLoc import Odometry


class Localization:
    def __init__(self) -> None:
        self.odom = Odometry()
        self.rate = rospy.Rate(self.odom.f)
        self.run()

    def update_callback(self, msg):
        pass

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
