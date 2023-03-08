#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped, PoseStamped, PoseArray, Pose
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

# How to import modules from another package/directory https://roboticsbackend.com/ros-import-python-module-from-another-package/
from odometry.odometryFusion import OdometryFusion



class FastSLAM:
    def __init__(self,verbose=False) -> None:
        self.sub_aruco = rospy.Subscriber(
            "/aruco/markers", MarkerArray, self.aruco_callback
        )
        self.sub_anchor = rospy.Subscriber(
            "/aruco500/markers", MarkerArray, self.anchor_callback
        )

        self.particle_publisher = rospy.Publisher("/particlecloud", PoseArray,queue_size=10)

        self.verbose = verbose
        # self.odom = OdometryCustom()
        self.odom = OdometryFusion(SLAM=True)

        self.rate = rospy.Rate(self.odom.f)
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

        #Init static TF between odom and base_link in order to find correct transform before odometry is init
        t = TransformStamped()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.header.stamp = rospy.Time.now()
        t.transform.translation.x = 0
        t.transform.translation.y = 0
        t.transform.translation.z = 0
        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1
        self.brStatic.sendTransform(t)

        rospy.sleep(1)
        #init static TF between map and odom
        # t = TransformStamped()
        # t.header.frame_id = "map"
        # t.child_frame_id = "odom"
        # t.header.stamp = rospy.Time.now()
        # t.transform.translation.x = 0
        # t.transform.translation.y = 0
        # t.transform.translation.z = 0
        # t.transform.rotation.x = 0
        # t.transform.rotation.y = 0
        # t.transform.rotation.z = 0
        # t.transform.rotation.w = 1
        # self.brStatic.sendTransform(t)

        #fastSLAM stuff
        self.SLAMinitialized = False
        self.M = 15 #Number of particles

        #self.J = 4 #Number of landmarks
        # init_uncertainty = 1000 #large uncertainty, since they're all put in the origin
        # default_landmark = np.zeros(3,4)
        # default_landmark[0,1] = init_uncertainty
        # default_landmark[1,2] = init_uncertainty
        # self.particles = np.zeros((self.M,3,1+self.J*4)) #particle m: [x;y;theta],[landmark1x;landmark1y,0,],[[landmark1sigmax;0;0],[0;landmark1sigmay;0],[0;0;0]],...,[landmarkJx;landmarkJy,0,],[[landmarkJsigmax;0;0],[0;landmarkJsigmay;0],[0;0;0]]]
        # self.particles[:,:,1:] = default_landmark #broadcasting should repeat default_landmark for all landmarks
        
        self.particles = np.zeros((self.M,3,1)) #init particles with only x,y,theta, no landmarks. They will be added when observed/seen
        self.seen_landmarks = [] #list of seen landmarks
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
                # if self.latest_stamp != t.header.stamp:
                #     if self.verbose:
                #         rospy.logdebug(f"Publishing transform from {t.header.frame_id} to {t.child_frame_id}")
                #     self.brStatic.sendTransform(t)
                #     self.latest_stamp = t.header.stamp
                #     self.latest_t = t
                if not self.SLAMinitialized:
                    self.particles[:,:,0] = np.array([t.transform.translation.x, t.transform.translation.y, self._quaternion_to_yaw(t.transform.rotation)])
                    self.SLAMinitialized = True


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
    
    def _particle_to_pose(self,x,y,yaw):
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = 0
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, yaw)
        pose.orientation.x = q[0]
        pose.orientation.y = q[1]
        pose.orientation.z = q[2]
        pose.orientation.w = q[3]
        return pose
    
    def run(self):
        while not rospy.is_shutdown():
            #rospy.logdebug("Inside run")
            #-- from localization.py
            self.place_anchor() #anchor placed and init position is known

            #self.predict()
            
            if self.SLAMinitialized:
                particleCloud = MarkerArray()
                particleCloud.header.frame_id = "map"
                particleCloud.header.stamp = rospy.Time.now()

            #---new fastSLAM
            #sample M particles using prior distribution from odometryFusion
                for k in range(self.particles.shape[0]):
                    x,y,yaw = self.odom.particleStep(self.particles[k,0,0],self.particles[k,1,0],self.particles[k,2,0])
                    self.particles[k,0,0] = x
                    self.particles[k,1,0] = y
                    self.particles[k,2,0] = yaw
                    particleCloud.poses.append(self._particle_to_pose(x,y,yaw))
            #for particle 
                #for each observed landmark
                    #if not seen before
                        #init landmark
                        #default importance weight
                    #else
                        #EKF update for each landmark
                        #update importance weight
                #for each UNobserved landmark
                    #leave landmark as is/unchanged
            
            #resample particles

            #return map and localization of best importance weight particle
            self.particle_publisher.publish(particleCloud)
            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("fastSLAM",log_level=rospy.DEBUG)
    FastSLAM()
    rospy.spin()
