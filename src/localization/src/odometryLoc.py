#!/usr/bin/env python
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
import tf_conversions
import tf2_ros
import math
from nav_msgs.msg import Odometry

class OdometryCustom:
    def __init__(self) -> None:
        self.sub_goal = rospy.Subscriber(
            "/motor/encoders", Encoders, self.encoder_callback
        )
        self.odom_publisher = rospy.Publisher("/odometry", Odometry)
        self.ticks_per_rev = 3072
        self.r = 0.04921
        self.B = 0.3
        self.f = 20
        self.encoders = Encoders()

        self.x = 0
        self.y = 0
        self.yaw = 0
        self.old_stamp = Encoders().header.stamp #Init empty stamp

        #self.rate = rospy.Rate(self.f)
        #self.run()

    def encoder_callback(self, msg):
        self.encoders = msg

    def translate_encoders(self):
        """
        returns V*dt and w*dt
        """

        delta_right = (
            self.encoders.delta_encoder_right * 2 * math.pi / self.ticks_per_rev
        )
        delta_left = self.encoders.delta_encoder_left * 2 * math.pi / self.ticks_per_rev

        vdt = self.r * (delta_right + delta_left) / 2
        wdt = self.r * (delta_right - delta_left) / self.B

        return vdt, wdt

    def step(self):
        #while not rospy.is_shutdown():
        br = tf2_ros.TransformBroadcaster()
        odom = Odometry()

        t = TransformStamped()
        t.header.frame_id = "odom"
        t.header.stamp = self.encoders.header.stamp
        t.child_frame_id = "base_link"

        # TODO: Fill in
        vdt, wdt = self.translate_encoders()

        self.x += vdt * math.cos(self.yaw)
        self.y += vdt * math.sin(self.yaw)
        self.yaw += wdt

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
        
        #self.rate.sleep()

    def get_pose(self):
        return self.x, self.y, self.yaw

