#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import math
from nav_msgs.msg import Odometry
import numpy as np

class EncoderState:
    def __init__(self,encoders) -> None:
        # self.sub_encoder = rospy.Subscriber(
        #     "/motor/encoders", Encoders, self.encoder_callback
        # )
        # self.odom_publisher = rospy.Publisher("/odometry", Odometry)
        self.ticks_per_rev = 3072
        self.r = 0.04921
        self.B = 0.3
        self.f = 20
        self.encoders = encoders
        self.dt = 1/self.f
        self.K = 2*np.pi/self.ticks_per_rev

        self.v = None
        self.w = None

        #Estimation 
        self.G = np.eye(2)
        self.R = np.eye(2) #TODO Fill this in
        self.R[0,0] = 10
        self.R[1,1] = 100



    # def encoder_callback(self, msg):
    #     self.encoders = msg
    #     self.encoder2velocities()

    def encoder2velocities(self):
        """
        returns V and w
        """
        # if not encoders_msg:
        #     encoders_msg = self.encoders

        encoders_msg = self.encoders

        self.v = self.r*self.K*(encoders_msg.delta_encoder_right+encoders_msg.delta_encoder_left)/(2*self.dt)
        self.w = self.r*self.K*(encoders_msg.delta_encoder_right-encoders_msg.delta_encoder_left)/(self.B*self.dt)




