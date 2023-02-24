#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import math
from nav_msgs.msg import Odometry
import numpy as np
from sensor_msgs.msg import Imu
from estStateEncoder import EncoderState

class IMUState:
    def __init__(self,imuData) -> None:
        # self.sub_imu = rospy.Subscriber(
        #     "/imu/data", Imu, self.imu_callback
        # )
        # self.odom_publisher = rospy.Publisher("/odometry", Odometry)
        
        #self.imu = None

        self.dt = EncoderState(None).dt
        self.v = None
        self.w = None

        #Estimation 
        self.H = np.eye(2)
        self.H[1,1] = 1/self.dt
        self.Q = np.eye(2)*1 # TODO Fill this in

        self.z_acc_x = imuData.linear_acceleration.x
        self.z_gyro_z = imuData.angular_velocity.z
        self.z_acc_x_hat = None
        self.z_gyro_z_hat = None

    # def imu_callback(self, msg):
    #     self.imu = msg
    #     self.z_acc_x = msg.linear_acceleration.x
    #     self.z_gyro_z = msg.angular_velocity.z
    #     self.velocities2imuData()

    def velocities2imuData(self,mu):
        """
        calculates predicted measurements from v and w
        """
        #if not imu_msg:
            # imu_msg = self.imu
        
        v = mu[0]
        w = mu[1]

        self.z_acc_x_hat = v/self.dt
        self.z_gyro_z_hat = w



