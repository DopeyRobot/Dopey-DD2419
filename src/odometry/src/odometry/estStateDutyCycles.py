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
        pass
        # self.sub_goal = rospy.Subscriber(
        #     "/motor/encoders", Encoders, self.encoder_callback
        # )
        # self.odom_publisher = rospy.Publisher("/odometry", Odometry)
        

        #self.rate = rospy.Rate(self.f)
        #self.run()

    # def encoder_callback(self, msg):
    #     self.encoders = msg


    def step(self):
        pass
        #while not rospy.is_shutdown():
        


        #to avoid redundat tf warnings
       
        
        #self.rate.sleep()



