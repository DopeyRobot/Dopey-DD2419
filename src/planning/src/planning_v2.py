#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, TwistStamped, PoseStamped
from math import atan2, sqrt


class planning_v2():

    def __init__(self):    
        print('init')
        self.f = 10
        self.rate = rospy.Rate(self.f)
        self.timeout = rospy.Duration(2)
        self.twist = TwistStamped()
        ## Max linear velocity (m/s)
        self.max_linear_velocity = 0.05
        ## Max angular velocity (rad/s)
        self.max_angular_velocity = 0.05
            


        sub_odom = rospy.Subscriber("/motor/encoders", Odometry, self.get_odom)
        sub_goal = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.get_goal) 
        self.pub_twist = rospy.Publisher("/motor_controller/twist", TwistStamped, queue_size = 10)

        self.x = 0.0
        self.y = 0.0 
        self.theta = 0.0
        self.goal = PoseStamped()

        self.run()


    def get_odom(self, msg):
        print('odom')
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([self.rot_q.x, self.rot_q.y, self.rot_q.z, self.rot_q.w])


    def get_goal(self, msg):
        print('goal')
        self.goal = msg
        self.goal.pose.position.x = msg.pose.position.x
        self.goal.pose.position.y = msg.pose.position.y


    def run(self):

        while not rospy.is_shutdown():
            
            inc_x = self.goal.pose.position.x -self.x
            inc_y = self.goal.pose.position.y -self.y

            angle_to_goal = atan2(inc_y, inc_x)
            desired_heading = angle_to_goal - self.theta

            self.twist.twist.angular.z = 4 * desired_heading
            if self.twist.twist.angular.z > self.max_angular_velocity:
                self.twist.twist.angular.z = self.max_angular_velocity

            self.twist.twist.linear.x = 0.5 * sqrt(inc_x ** 2 + inc_y ** 2)
            if self.twist.twist.linear.x > self.max_linear_velocity:
                self.twist.twist.linear.x = self.max_linear_velocity
            
            self.pub_twist.publish(self.twist)

        self.rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node("planning_v2") 
        planning_v2()      
        rospy.spin()
    except rospy.ROSInterruptException:
        pass






'''            if abs(desired_heading) > 0.1:
                self.twist.twist.linear.x = 0.0
                self.twist.twist.angular.z = 0.3 * desired_heading / abs(desired_heading)
                self.pub_twist.publish(self.twist)
            else:
                self.twist.twist.linear.x = 0.5
                self.twist.twist.angular.z = 0.0
                self.pub_twist.publish(self.twist)'''
        