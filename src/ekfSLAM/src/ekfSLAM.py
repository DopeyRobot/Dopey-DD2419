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
from geometry_msgs.msg import TwistStamped
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

        self.sub_odom_state = rospy.Subscriber(
            "/odometry/curr_vel_state", TwistStamped, self.odom_state_callback)



        self.verbose = verbose
        f = EncoderState(None).f
        self.rate = rospy.Rate(f) #=20
        self.dt = 1/f

        #publish stuf
        self.old_stamp = TwistStamped().header.stamp #Init empty stamp
        self.currHeaderStamp = None
        self.startOK = False

        #SLAM stuff
        self.v = None #predicted x velocity from odom sensor fusion
        self.w = None #predicted z angular velocity from odom sensor fusion'


        self.mu_t = np.zeros((3,1)) #[x,y,yaw]
        self.sigma_t = np.zeros((3,3)) #covariance matrix
        self.mu_bar_t = None
        self.sigma_bar_t = None

        self.F = np.eye(3) #state transition matrix, will grow with 3*#landmarks column wise
        self.R = np.eye(3) #process noise matrix
        self.Q = np.eye(3) #measurement noise matrix
        

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

    
    def odom_state_callback(self,data):
        # rospy.loginfo("inside odom_state callback")
        self.v = data.twist.linear.x #predicted x velocity from odom sensor fusion
        self.w = data.twist.angular.z #predicted z angular velocity from odom sensor fusion'
        if np.isclose(data.twist.angular.z,0):
            self.w = 1e-5
        self.currHeaderStamp = data.header.stamp
        self.startOK = True

    def sendCurrentTransform(self):
        x = self.mu_t[0,0]
        y = self.mu_t[1,0]
        yaw = self.mu_t[2,0]

        br = tf2_ros.TransformBroadcaster()

        t = TransformStamped()
        t.header.frame_id = "odom"
        t.header.stamp = self.currHeaderStamp
        t.child_frame_id = "base_link"

        t.transform.translation.x = x
        t.transform.translation.y = y
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        #to avoid redundat tf warnings
        if self.old_stamp != t.header.stamp:
            br.sendTransform(t)
            self.old_stamp = t.header.stamp



    def run(self):
        while not rospy.is_shutdown():
            if self.startOK:
                # self.mu_bar_t = self.mu_t + self.F.T @ np.array([[self.v*np.cos(self.mu_t[2,0])],[self.v*np.sin(self.mu_t[2,0])],[self.w*self.dt]])  #predicted state according to odometry fusion - not like the book        
                # G = np.eye(self.F.shape[1]) + self.F.T @ np.array([[0,0,-self.v*np.sin(self.mu_t[2,0])],[0,0,self.v*np.cos(self.mu_t[2,0])],[0,0,0]]) @ self.F
                vOw = self.v/self.w
                self.mu_bar_t = self.mu_t + self.F.T @ np.array([[-vOw*np.sin(self.mu_t[2,0]) + vOw*np.sin(self.mu_t[2,0] + self.w*self.dt)],[vOw*np.cos(self.mu_t[2,0]) - vOw*np.cos(self.mu_t[2,0] + self.w*self.dt)],[self.w*self.dt]])
                G = np.eye(self.F.shape[1]) + self.F.T @ np.array([[0,0,-vOw*np.cos(self.mu_t[2,0]) + vOw*np.cos(self.mu_t[2,0] + self.w*self.dt)],[0,0,-vOw*np.sin(self.mu_t[2,0]) + vOw*np.sin(self.mu_t[2,0] + self.w*self.dt)],[0,0,0]]) @ self.F
                self.sigma_bar_t = G @ self.sigma_t @ G.T  + self.F.T @ self.R @ self.F


                self.mu_t = self.mu_bar_t
                self.sigma_t = self.sigma_bar_t
                self.sendCurrentTransform()
                self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("ekfSLAM",log_level=rospy.DEBUG)
    EkfSLAM()
    rospy.spin()
