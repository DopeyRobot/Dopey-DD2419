#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
from nav_msgs.msg import Odometry
import tf_conversions
import tf2_ros
import math


class OdometryFusion:
    def __init__(self) -> None:
        # self.sub_goal = rospy.Subscriber(
        #     "/motor/encoders", Encoders, self.encoder_callback
        # )
        # self.odom_publisher = rospy.Publisher("/odometry", Odometry)
        

        self.rate = rospy.Rate(self.f)
        self.run()
        self.mu_pred_t = None
        self.sigma_pred_t = None
        self.mu_t = None
        self.sigma_t = None

    # def encoder_callback(self, msg):
    #     self.encoders = msg

    def predict(self):
        """
        calculates the predicted state and covariance from duty cycles
        """
        pass


    def run(self):
        while not rospy.is_shutdown():
            #mu_pred_t, sigma_pred_t = self.predict()
            #predict step
            #mu_pred_t, sigma_pred_t <- dutyCycles2velocities
            #update step 1 - Encoder
            #K1
            #mu_t,sigma_t <-

            #update step 2 - IMU
            #K2





            self.rate.sleep()


if __name__ == "__main__":
    rospy.init_node("odometryFusion")
    OdometryFusion()
    rospy.spin()
