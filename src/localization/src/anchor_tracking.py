#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, PoseStamped
from aruco_msgs.msg import MarkerArray
import tf_conversions
import tf2_ros
import math
import tf2_geometry_msgs
#from odometryLoc import OdometryCustom

from scipy.spatial.transform import Rotation as R
import numpy as np
import pdb

## Import module from other package/directory
#import sys
#sys.path.append("/home/robot/dopey_ws/src/odometry") #this needs to be changed for the actual Dopey Robot
#from odometryFusion import OdometryFusion







class AnchorTracking:
    def __init__(self,verbose=False) -> None:
        self.sub_anchor = rospy.Subscriber(
            "/aruco/markers", MarkerArray, self.anchor_callback
        )
        self.verbose = verbose
        # self.odom = OdometryCustom()

        self.rate = rospy.Rate(20)
        #Anchor stuff
        self.anchor = None #In aruco_frame TF
        self.first_anchor = None
        self.anchorID = 500
        #self.test = OdometryPublisher()
        #TF stuff
        self.aruco_frame = "camera_color_optical_frame"
        self.buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.brStatic = tf2_ros.StaticTransformBroadcaster()
        self.br = tf2_ros.TransformBroadcaster()
        self.latest_stamp = PoseStamped().header.stamp #Init empty stamp
        self.latest_stamp2 = PoseStamped().header.stamp #Init empty stamp
        self.sendOld = False
        self.latest_t = None


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
        
    def predict(self):
        self.odom.step()
    
    def place_anchor(self):
        if self.anchor:
            transform_is_possible = self.buffer.can_transform("odom", self.aruco_frame, self.anchor.header.stamp, rospy.Duration(2))
            
            anchorPoseStamped = self._PoseWithCovarianceStamped_to_PoseStamped(self.anchor)
            if transform_is_possible:
                
                #transform aruco_frame to odom
                transform2odom = self.buffer.lookup_transform("odom", self.anchor.header.frame_id, self.anchor.header.stamp, rospy.Duration(20))
                
                anchor_odom_pose = tf2_geometry_msgs.do_transform_pose(anchorPoseStamped, transform2odom)
                
                #transform odom to map
                t = TransformStamped()
                t.header.frame_id = "odom"
                t.child_frame_id = "map"
                t.header.stamp = self.anchor.header.stamp

                t.transform.translation.x = anchor_odom_pose.pose.position.x
                t.transform.translation.y = anchor_odom_pose.pose.position.y
                t.transform.translation.z = anchor_odom_pose.pose.position.z

                t.transform.rotation.x = anchor_odom_pose.pose.orientation.x
                t.transform.rotation.y = anchor_odom_pose.pose.orientation.y
                t.transform.rotation.z = anchor_odom_pose.pose.orientation.z
                t.transform.rotation.w = anchor_odom_pose.pose.orientation.w
                
                #transform map to odom
                t = self._inverse_transform(t)
                #To avoid redudant tf warnings
                if self.latest_stamp != t.header.stamp:
                    if self.verbose:
                        rospy.logdebug(f"Publishing transform from {t.header.frame_id} to {t.child_frame_id}")
                    self.brStatic.sendTransform(t)
                    self.latest_stamp = t.header.stamp
                    self.latest_t = t


    def _inverse_transform(self, transform):
       
        inverse_transform = transform
        old_parent = transform.header.frame_id
        old_child = transform.child_frame_id
        inverse_transform.header.frame_id = old_child
        inverse_transform.child_frame_id = old_parent
        r = R.from_quat([transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z, transform.transform.rotation.w])
        t = np.array([transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z])

        
        T = np.zeros((4,4))
        T[:3,:3] = r.as_matrix()
        T[:3,3] = t
        T[3,3] = 1
        inversed_T = np.linalg.inv(T)
        
        inversed_r = R.from_matrix(inversed_T[:3,:3])
        inversed_t = inversed_T[:3,3]
        inverse_transform.transform.rotation.x = inversed_r.as_quat()[0]
        inverse_transform.transform.rotation.y = inversed_r.as_quat()[1]
        inverse_transform.transform.rotation.z = inversed_r.as_quat()[2]
        inverse_transform.transform.rotation.w = inversed_r.as_quat()[3]
        inverse_transform.transform.translation.x = inversed_t[0]
        inverse_transform.transform.translation.y = inversed_t[1]
        inverse_transform.transform.translation.z = inversed_t[2]
        
        return inverse_transform

    def _PoseWithCovarianceStamped_to_PoseStamped(self, PoseWithCovarianceStamped):
        PoseS = PoseStamped()
        PoseS.pose = PoseWithCovarianceStamped.pose.pose
        PoseS.header = PoseWithCovarianceStamped.header
        return PoseS
    
    def run(self):
        while not rospy.is_shutdown():
            #rospy.logdebug("Inside run")
            self.place_anchor()
            self.rate.sleep()
        

if __name__ == "__main__":
    rospy.init_node("anchor_tracking",log_level=rospy.DEBUG)
    AnchorTracking()
    rospy.spin()
