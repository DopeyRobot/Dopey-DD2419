#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose
from aruco_msgs.msg import MarkerArray
import tf_conversions
import tf2_ros
import math
import tf2_geometry_msgs
#from odometryLoc import OdometryCustom
import tf

from scipy.spatial.transform import Rotation as R
import numpy as np
import pdb

# ## Import module from other package/directory

# # How to import modules from another package/directory https://roboticsbackend.com/ros-import-python-module-from-another-package/
from odometry.estStateEncoder import EncoderState



class EkfSLAM:
    def __init__(self,verbose=False) -> None:
        self.sub_aruco = rospy.Subscriber(
            "/aruco/markers", MarkerArray, self.aruco_callback
        )
        self.sub_anchor = rospy.Subscriber(
            "/aruco500/markers", MarkerArray, self.anchor_callback
        )


        self.verbose = verbose
        self.rate = EncoderState(None).f #=20

        #SLAM stuff

        self.mu_t = None
        self.sigma_t = None
        self.mu_bar_t = None
        self.sigma_bar_t = None
        

        self.run()

    def anchor_callback(self, msg):
        #rospy.logdebug("anchor callback")
        for marker in msg.markers:
            if marker.id == self.anchorID:
                anchor = PoseWithCovarianceStamped()
                anchor.pose.pose = marker.pose.pose #.pose & .covariance exist after this
                anchor.pose.covariance = marker.pose.covariance
                anchor.header.frame_id = self.aruco_frame
                anchor.header.stamp = msg.header.stamp

                if self.first_anchor is None: #to keep the first anchor coordinates only
                    self.first_anchor = anchor
                self.anchor = anchor
    
    def aruco_callback(self,msg):
        pass

    
    
    def run(self):
        while not rospy.is_shutdown():
        
            
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("ekfSLAM",log_level=rospy.DEBUG)
    EkfSLAM()
    rospy.spin()
