#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
from nav_msgs.msg import Odometry
import tf_conversions
import tf2_ros
import math

from estStateIMU import IMUState
from estStateEncoder import EncoderState
from sensor_msgs.msg import Imu

import numpy as np

class OdometryFusion:
    def __init__(self) -> None:
        self.odom_publisher = rospy.Publisher("/odometry", Odometry)

        self.sub_encoder = rospy.Subscriber(
            "/motor/encoders", Encoders, self.encoder_callback
        )

        self.sub_imu = rospy.Subscriber(
            "/imu/data", Imu, self.imu_callback
        )

        self.imu = None
        self.encoders = None

        #self.rate = rospy.Rate(self.f)
        #self.run()
        self.mu_pred_t = None #TODO init
        self.sigma_pred_t = None #TODO init
        self.mu_t = np.array([0,0]) #TODO init
        self.sigma_t = np.eye(2)*10 #TODO init
        self.f = EncoderState(None).f

        
    
    def encoder_callback(self, msg):
        self.encoders = msg
        #self.encoder2velocities()

    def imu_callback(self, msg):
        self.imu = msg
        # self.z_acc_x = msg.linear_acceleration.x
        # self.z_gyro_z = msg.angular_velocity.z
        #self.velocities2imuData()


    def fusion(self):
        encoderState = EncoderState(self.encoders)
        encoderState.encoder2velocities()
        imuState = IMUState(self.imu)

        #predict
        self.mu_pred_t = np.array([encoderState.v,encoderState.w])
        self.sigma_pred_t = self.sigma_t + encoderState.R

        #update
        imuState.velocities2imuData(self.mu_pred_t)
        K = self.sigma_pred_t @ imuState.H.T @ np.linalg.inv(imuState.H @ self.sigma_pred_t @ imuState.H.T + imuState.Q) #TODO: Optimize with einsum
        self.mu_t = self.mu_pred_t + K @ (np.array([imuState.z_acc_x,imuState.z_gyro_z]) - np.array([imuState.z_acc_x_hat,imuState.z_gyro_z_hat]))
        self.sigma_t = (np.eye(2) - K @ imuState.H) @ self.sigma_pred_t
        
        return self.mu_t[0],self.mu_t[1]


    def step(self):

        # TODO: Fill in
        v,w = self.fusion()
        vdt = v*self.dt
        wdt = w*self.dt

        self.x += vdt * math.cos(self.yaw)
        self.y += vdt * math.sin(self.yaw)
        self.yaw += wdt

        #----
        br = tf2_ros.TransformBroadcaster()
        odom = Odometry()

        t = TransformStamped()
        t.header.frame_id = "odom"
        t.header.stamp = self.encoders.header.stamp
        t.child_frame_id = "base_link"
        
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        q = tf_conversions.transformations.quaternion_from_euler(0, 0, self.yaw)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        odom.child_frame_id = "base_link"
        odom.twist.twist.linear.x = vdt*math.cos(self.yaw)/self.f
        odom.twist.twist.linear.y = vdt*math.sin(self.yaw)/self.f
        odom.twist.twist.angular.z = wdt/self.f

        self.odom_publisher.publish(odom)


        #to avoid redundat tf warnings
        if self.old_stamp != t.header.stamp:
            br.sendTransform(t)
            self.old_stamp = t.header.stamp


if __name__ == "__main__":
    rospy.init_node("odometryFusion")
    OdometryFusion()
    rospy.spin()
