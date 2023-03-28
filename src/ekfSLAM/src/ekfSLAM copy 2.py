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
from scipy.linalg import block_diag
import numpy as np
import pdb
from geometry_msgs.msg import TwistStamped
# ## Import module from other package/directory

# # How to import modules from another package/directory https://roboticsbackend.com/ros-import-python-module-from-another-package/
from odometry.estStateEncoder import EncoderState

#TODO: find reason why landmarks are published to middle of base_link
#1. Keep odometry separate from the update step
#2. Do all odometry stuff on baselink in odom frame as it is now
#3. All updates on mu_bar[0:3] should be implemented on the trasnform from odom to map and keep the base_link untouched

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


        #
        self.verbose = verbose
        f = EncoderState(None).f
        self.rate = rospy.Rate(f) #=20
        self.dt = 1/f

        #aruco stuff
        self.anchorID = 500
        self.aruco_frame = "camera_color_optical_frame"
        self.buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.anchor_landmarks = []
        self.anchor_stamp = None
        self.landmarks = []
        self.landmarks_stamp = None

        #publish stuff
        self.old_stamp = TwistStamped().header.stamp #Init empty stamp
        self.old_stamps4landmarks = []
        self.currHeaderStamp = None
        self.startOK = False

        #SLAM stuff
        self.v = None #predicted x velocity from odom sensor fusion
        self.w = None #predicted z angular velocity from odom sensor fusion'


        self.mu_t = np.zeros((3,1)) #[x,y,yaw]
        self.sigma_t = np.zeros((3,3)) #covariance matrix
        self.mu_bar_t = None
        self.sigma_bar_t = None

        self.Fx = np.eye(3) #state transition matrix, will grow with 3*#landmarks column wise
        self.R = np.eye(3) #process noise matrix
        self.Q = np.eye(3)*1e-2 #measurement noise matrix
        
        self.seenLandmarks = []#np.empty((1,0), int)#np.array([]) #List that tracks seen landmarks, keeps track of arucoID

        self.run()

    def anchor_callback(self, msg):
        #rospy.logdebug("anchor callback")
        for marker in msg.markers:
            poseWithCov = PoseWithCovarianceStamped()
            poseWithCov.pose.pose = marker.pose.pose 
            poseWithCov.pose.covariance = marker.pose.covariance
            poseWithCov.header.frame_id = self.aruco_frame
            poseWithCov.header.stamp = msg.header.stamp
            if marker.id == self.anchorID:
                #add marker.id to seen landmarks only if it's not already there
                if marker.id not in self.seenLandmarks:
                    self.add_landmark(marker.id,self._aruco_in_frame(poseWithCov,parent_frame="odom"))
                else:
                    self.update_landmark(marker.id,self._aruco_in_frame(poseWithCov,parent_frame="odom"))
            
                
    
    def aruco_callback(self,msg):
        for marker in msg.markers:
            poseWithCov = PoseWithCovarianceStamped()
            poseWithCov.pose.pose = marker.pose.pose 
            poseWithCov.pose.covariance = marker.pose.covariance
            poseWithCov.header.frame_id = self.aruco_frame
            poseWithCov.header.stamp = msg.header.stamp
            if marker.id != self.anchorID:
                # pdb.set_trace()
                if marker.id not in self.seenLandmarks:
                    self.add_landmark(marker.id,self._aruco_in_frame(poseWithCov,parent_frame="odom"))
                else:
                    self.update_landmark(marker.id,self._aruco_in_frame(poseWithCov,parent_frame="odom"))
                    # rospy.logdebug(marker.pose.pose.position.x)

    # def anchor_callback(self, msg):
    #     rospy.logdebug("anchor callback")
    #     for marker in msg.markers:
    #         if marker.id == self.anchorID:
    #             #add marker.id to seen landmarks only if it's not already there
    #             #if msg.header.stamp == self.currHeaderStamp:
    #             rospy.logdebug("adding anchor landmark")
    #             self.anchor_landmarks = np.array([])
    #             # self.anchor_landmarks.append(marker)
    #             np.append(self.anchor_landmarks,marker)
    #             self.anchor_stamp = msg.header.stamp
                
    
    # def aruco_callback(self,msg):
    #     rospy.logdebug("aruco callback")
    #     for marker in msg.markers:
    #         if marker.id != self.anchorID:
    #             #if msg.header.stamp == self.currHeaderStamp:
    #             rospy.logdebug("adding landmark")
    #             self.landmarks = np.array([])
    #             # self.landmarks.append(marker)
    #             np.append(self.anchor_landmarks,marker)
    #             self.landmarks_stamp = msg.header.stamp

    
    def add_landmark(self,arucoID,pose):
        rospy.logdebug("adding landmark")
        # pdb.set_trace()
        self.seenLandmarks.append(arucoID)
        # np.append(self.seenLandmarks,arucoID)
        # self.seenLandmarks += np.array([arucoID])
        # a = np.empty((1,0), int)
        # np.append(a,np.array([arucoID]),axis=1)
        # rospy.logdebug(a)
        # rospy.logdebug(arucoID)
        # rospy.logdebug(self.seenLandmarks)

        self._inflate_matrices()
        # q = pose.pose.orientation
        # yaw = tf_conversions.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        self.mu_bar_t[-3:,:] = np.array([[pose.pose.position.x],[pose.pose.position.y],[0]])
        #self.sigma_bar_t[-3:,-3:] = self.sigma_bar_t[:3,:3] #Inherit covariance from robot pose when seen
        # pdb.set_trace()

    def update_landmark(self,arucoID,pose):
        rospy.logdebug("updating landmark")
        # pdb.set_trace()
        # q = pose.pose.orientation
        # yaw = tf_conversions.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        z = np.array([[pose.pose.position.x-self.mu_bar_t[0,0]],[pose.pose.position.y-self.mu_bar_t[1,0]],[0]])
        j = self.seenLandmarks.index(arucoID)
        N = len(self.seenLandmarks)#.shape[0]
        # z_hat = np.array([[self.mu_bar_t[0,j]-self.mu_bar_t[0,0]],[self.mu_bar_t[1,j]-self.mu_bar_t[1,0]],[0]])
        z_hat = np.array([[self.mu_bar_t[3+3*j,0]-self.mu_bar_t[0,0]],[self.mu_bar_t[4+3*j,0]-self.mu_bar_t[1,0]],[0]])
        H_prime = np.zeros((3,6))
        H_prime[0,0] = -1
        H_prime[0,3] = 1
        H_prime[1,1] = -1
        H_prime[1,4] = 1
        Fxj = np.zeros((6,3+3*N))
        Fxj[:3,:3] = np.eye(3)
        Fxj[3:,3+3*j:3+3*(j+1)] = np.eye(3)
        H = H_prime @ Fxj

        K = self.sigma_bar_t @ H.T @ np.linalg.inv(H @ self.sigma_bar_t @ H.T + self.Q)
        self.mu_bar_t = self.mu_bar_t + K @ (z-z_hat)
        KH= K @ H
        assert KH.shape[0] == KH.shape[1]
        self.sigma_bar_t = (np.eye(KH.shape[0]) - KH) @ self.sigma_bar_t
        # pdb.set_trace()

    def _inflate_matrices(self):
            inf=1e9
            self.old_stamps4landmarks.append(None)
            #add 3 columns to F
            self.Fx = np.hstack((self.Fx,np.zeros((3,3))))
            #add 3 rows to F
            #self.Fx = np.vstack((self.Fx,np.zeros((3,self.Fx.shape[1]))))
            #set bottom right 3x3 to identity
            self.Fx[-3:,-3:] = np.eye(3)
            #add 3 rows to mu and mu_bar
            self.mu_t = np.vstack((self.mu_t,np.zeros((3,1))))
            self.mu_bar_t = np.vstack((self.mu_bar_t,np.zeros((3,1))))
            #add covariance matrix space for new landmark on sigma and sigma_bar
            # self.sigma_t = np.hstack((self.sigma_t,np.eye(3)*inf))
            # self.sigma_bar_t =  np.hstack((self.sigma_bar_t,np.eye(3)*inf))
            #stack matrices on diagonal
            self.sigma_t = block_diag(self.sigma_t,np.eye(3)*inf)
            self.sigma_bar_t = block_diag(self.sigma_bar_t,np.eye(3)*inf)

    
    def _aruco_in_frame(self,pose,parent_frame):
        #if self.buffer.can_transform(parent_frame, self.aruco_frame, self.currHeaderStamp, rospy.Duration(2)):
        pdb.set_trace()
        poseStamped = self._PoseWithCovarianceStamped_to_PoseStamped(pose)
        transform2odom = self.buffer.lookup_transform(parent_frame, self.aruco_frame, self.currHeaderStamp, rospy.Duration(20))
        odom_pose = tf2_geometry_msgs.do_transform_pose(poseStamped, transform2odom) #where the marker is in odom frame
        pdb.set_trace()
        
        return odom_pose
        # else:
        #     raise Exception("No transform from aruco frame to odom frame")
        


    def _PoseWithCovarianceStamped_to_PoseStamped(self, PoseWithCovarianceStamped):
        PoseS = PoseStamped()
        PoseS.pose = PoseWithCovarianceStamped.pose.pose
        PoseS.header = PoseWithCovarianceStamped.header
        return PoseS
    
    def odom_state_callback(self,data):
        # rospy.loginfo("inside odom_state callback")
        self.v = data.twist.linear.x #predicted x velocity from odom sensor fusion
        self.w = data.twist.angular.z #predicted z angular velocity from odom sensor fusion'
        if np.isclose(data.twist.angular.z,0):
            self.w = 1e-9
        self.currHeaderStamp = data.header.stamp
        self.startOK = True

    def sendCurrentRobotPose(self):
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
    
    def sendCurrentLandmarkPoses(self):
        br = tf2_ros.TransformBroadcaster()
        N = int((self.mu_t.shape[0]-3)/3)
        for i in range(N):
            t = TransformStamped()
            t.header.frame_id = "odom"
            t.header.stamp = self.currHeaderStamp
            t.child_frame_id = "landmark"+str(i+1)
            

            t.transform.translation.x = self.mu_t[3+3*i,0]
            t.transform.translation.y = self.mu_t[4+3*i,0]
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.mu_t[5+3*i,0])
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            #to avoid redundat tf warnings
            # rospy.logdebug(self.old_stamps4landmarks)
            # rospy.loginfo(self.old_stamps4landmarks[i] == t.header.stamp)
            if self.old_stamps4landmarks[i] != t.header.stamp:
                br.sendTransform(t)
                # rospy.logdebug("Sending transform of landmark"+str(i+1))
                self.old_stamps4landmarks[i] = t.header.stamp
    
    def odometry_prediction(self):
        vOw = self.v/self.w
        self.mu_bar_t = self.mu_t + self.Fx.T @ np.array([[-vOw*np.sin(self.mu_t[2,0]) + vOw*np.sin(self.mu_t[2,0] + self.w*self.dt)],[vOw*np.cos(self.mu_t[2,0]) - vOw*np.cos(self.mu_t[2,0] + self.w*self.dt)],[self.w*self.dt]])
        G = np.eye(self.Fx.shape[1]) + self.Fx.T @ np.array([[0,0,-vOw*np.cos(self.mu_t[2,0]) + vOw*np.cos(self.mu_t[2,0] + self.w*self.dt)],[0,0,-vOw*np.sin(self.mu_t[2,0]) + vOw*np.sin(self.mu_t[2,0] + self.w*self.dt)],[0,0,0]]) @ self.Fx
        self.sigma_bar_t = G @ self.sigma_t @ G.T  + self.Fx.T @ self.R @ self.Fx


    # def measurement_update(self):
    #     for marker in self.landmarks+self.anchor_landmarks:
    #         poseWithCov = PoseWithCovarianceStamped()
    #         poseWithCov.pose.pose = marker.pose.pose 
    #         poseWithCov.pose.covariance = marker.pose.covariance
    #         poseWithCov.header.frame_id = self.aruco_frame
    #         if marker.id == self.anchorID:
    #             poseWithCov.header.stamp = self.anchor_stamp
    #         else:
    #             poseWithCov.header.stamp = self.landmarks_stamp

    #         if marker.id not in self.seenLandmarks:
    #             self.add_landmark(marker.id,self._aruco_in_frame(poseWithCov,parent_frame="odom"))
    #         else:
    #             self.update_landmark(marker.id,self._aruco_in_frame(poseWithCov,parent_frame="odom"))

    def run(self):
        while not rospy.is_shutdown():
            if self.startOK:
                # self.mu_bar_t = self.mu_t + self.F.T @ np.array([[self.v*np.cos(self.mu_t[2,0])],[self.v*np.sin(self.mu_t[2,0])],[self.w*self.dt]])  #predicted state according to odometry fusion - not like the book        
                # G = np.eye(self.F.shape[1]) + self.F.T @ np.array([[0,0,-self.v*np.sin(self.mu_t[2,0])],[0,0,self.v*np.cos(self.mu_t[2,0])],[0,0,0]]) @ self.F
                # vOw = self.v/self.w
                # self.mu_bar_t = self.mu_t + self.Fx.T @ np.array([[-vOw*np.sin(self.mu_t[2,0]) + vOw*np.sin(self.mu_t[2,0] + self.w*self.dt)],[vOw*np.cos(self.mu_t[2,0]) - vOw*np.cos(self.mu_t[2,0] + self.w*self.dt)],[self.w*self.dt]])
                # G = np.eye(self.Fx.shape[1]) + self.Fx.T @ np.array([[0,0,-vOw*np.cos(self.mu_t[2,0]) + vOw*np.cos(self.mu_t[2,0] + self.w*self.dt)],[0,0,-vOw*np.sin(self.mu_t[2,0]) + vOw*np.sin(self.mu_t[2,0] + self.w*self.dt)],[0,0,0]]) @ self.Fx
                # self.sigma_bar_t = G @ self.sigma_t @ G.T  + self.Fx.T @ self.R @ self.Fx
                self.odometry_prediction()
                # self.measurement_update()
                self.mu_t = self.mu_bar_t
                self.sigma_t = self.sigma_bar_t
                # try:
                #     rospy.logdebug(self.mu_t[3])
                # except:
                #     pass
                

                # rospy.logdebug(self.mu_t.shape)
                # rospy.logdebug(self.mu_bar_t.shape)
                # rospy.logdebug(self.sigma_t)
                # rospy.logdebug(self.sigma_bar_t.shape)
                self.sendCurrentRobotPose()
                self.sendCurrentLandmarkPoses()

                # assert self.mu_t.shape == (3*len(self.seenLandmarks)+3,1)
                # assert self.mu_t.shape == self.mu_bar_t.shape
                # assert self.sigma_t.shape == (3*len(self.seenLandmarks)+3,3*len(self.seenLandmarks)+3)
                # assert self.sigma_t.shape == self.sigma_bar_t.shape

                self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("ekfSLAM",log_level=rospy.DEBUG)
    EkfSLAM()
    rospy.spin()
