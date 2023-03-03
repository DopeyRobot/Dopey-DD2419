#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from robp_msgs.msg import DutyCycles, Encoders
import math
import numpy as np


class CartesianController:
    def __init__(self, twist_topic: str = "/motor_controller/twist", verbose=False):
        self.duty_pub = rospy.Publisher("/motor/duty_cycles", DutyCycles, queue_size=10)
        self.encoder_sub = rospy.Subscriber(
            "/motor/encoders", Encoders, self.encoder_callback
        )
        self.twist_sub = rospy.Subscriber(twist_topic, Twist, self.twist_callback)

        self.sub_odom_state = rospy.Subscriber(
            "/odometry/curr_vel_state", Twist, self.odom_state_callback)

        self.f = 10
        self.b = 0.3
        self.r = 0.04921
        self.ticks_per_rev = 3072

        self.twist = Twist()
        self.encoders = Encoders()
        self.odom_vel_state = Twist()

        self.int_error_left = 0
        self.int_error_right = 0
        # P = 0.02
        # I = 0.05
        P = 0.02
        I = 0.07
        self.P_left = P*2
        self.I_left = I*1.1
        self.P_right = P
        self.I_right = I

        self.max_v = 0.35

        self.verbose = verbose

        # rospy.init_node("cartesian_controller", anonymous=True)
        rospy.loginfo("Cartesian controller started")

        self.rate = rospy.Rate(self.f)
        self.run()

    def run(self):
        while not rospy.is_shutdown():
            desired_w = -self.twist.angular.z
            desired_v = self.twist.linear.x

            if abs(desired_v) > self.max_v:
                desired_v = np.sign(desired_v)*self.max_v

            w_left, w_right = self.translate_encoders()

            desired_w_left = (self.b * desired_w + desired_v) / self.r
            desired_w_right = (-self.b * desired_w + desired_v) / self.r

            error_left = desired_w_left - w_left
            error_right = desired_w_right - w_right

            if self.verbose:
                rospy.loginfo(
                    "current error: {} and {}".format(error_left, error_right)
                )

            self.int_error_left += error_left * (1 / self.f)
            self.int_error_right += error_right * (1 / self.f)

            left_command = self.P_left * error_left + self.I_left * self.int_error_left
            right_command = (
                self.P_right * error_right + self.I_right * self.int_error_right
            )

            duty_left = np.clip(left_command, -1, 1)
            duty_right = np.clip(right_command, -1, 1)

            msg = DutyCycles()
            msg.duty_cycle_left = duty_left
            msg.duty_cycle_right = duty_right
            self.duty_pub.publish(msg)

            self.rate.sleep()

    def translate_encoders(self):

        w_left = (
            2 * math.pi * self.f * self.encoders.delta_encoder_left
        ) / self.ticks_per_rev
        w_right = (
            2 * math.pi * self.f * self.encoders.delta_encoder_right
        ) / self.ticks_per_rev

        return w_left, w_right
    
    def state_from_odom(self):
        v = self.odom_vel_state.linear.x #predicted x velocity from odom sensor fusion
        w = self.odom_vel_state.angular.z #predicted z angular velocity from odom sensor fusion
        w_left = (v-2*self.b*w)/self.r
        w_right = 2*self.b*w/self.r+w_left
        return w_left, w_right

    def encoder_callback(self, data):
        if self.verbose:
            rospy.loginfo("Encoder callback")
        self.encoders = data

        if self.verbose:

            rospy.loginfo(
                "Encoders received: {} and {}".format(
                    self.encoders.delta_encoder_left, self.encoders.delta_encoder_right
                )
            )

    def twist_callback(self, data):
        if self.verbose:
            rospy.loginfo("Twist callback")

        self.twist = data

        if self.verbose:
            rospy.loginfo("Twist received: {}".format(self.twist.twist.linear.x))

    def odom_state_callback(self,data):
        if self.verbose:
            rospy.loginfo("Odom velocity state callback")

        self.odom_vel_state = data

        if self.verbose:
            rospy.loginfo("Odom velocity state received: {}".format(self.twist.twist.linear.x))



if __name__ == "__main__":
    rospy.init_node("cartesian_controller", anonymous=True)
    controller = CartesianController("/motor_controller/twist")
    rospy.spin()
