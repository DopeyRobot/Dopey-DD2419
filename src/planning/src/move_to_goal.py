#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
from nav_msgs.msg import Path
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Point, TwistStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs
from nav_msgs.msg import Odometry
import math
#from rrt import path 

class move_to_goal():

    def __init__(self):

        self.Kp_ang = -0.05
        self.Ki_ang = -0.0003
        self.Kd_ang = 0.03
        self.Kp_dist = 0.06
        self.Ki_dist = 0.0002
        self_Kd_dist = 0.0
        self.integral_error_ang = 0.0
        self.integral_error_dist = 0.0
        self.prev_error_ang = 0.0
        self._prev_error_dist = 0.0
        self.threshold_ang = 0.2
        self.threshold_dist = 0.1
        self.error_ang = 0.0
        self.error_dist = 0.0
        self.goal_theta = 0.0
        self.odom_theta = 0.0
        self.goal_pose = PoseStamped()
        self.transformed_goal_pose = TransformStamped()
        self.transformed_goal_pose = PoseStamped() 
        self.twist = TwistStamped()  

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()

        self.f = 50
        self.rate = rospy.Rate(self.f)
        self.targetframe = "base_link"
        self.currentframe = "odom"
        self.timeout = rospy.Duration(2)
        self.odom = Odometry()

        self.publisher_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.publisher_twist = rospy.Publisher('motor_controller/twist', TwistStamped, queue_size=10)
        self.goal_subscriber = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.goal_callback) 
        self.odom_subscriber = rospy.Subscriber('odometry', Odometry, self.odom_callback) 
        self.run() 


    def goal_callback(self, msg):

        self.goal_pose.header.stamp = msg.header.stamp
        self.goal_pose.header.frame_id = msg.header.frame_id
        self.goal_pose.pose = msg.pose

        self.transformed_goal_pose = self.tf_buffer.transform(self.goal_pose, self.targetframe, rospy.Duration(1))

        rot_q = self.transformed_goal_pose.pose.orientation
        (_, _, self.goal_theta) = euler_from_quaternion([rot_q.x, rot_q.y, rot_q.z, rot_q.w])


        
        

    def odom_callback(self, msg):
        self.odom = msg
        odom_q = self.odom.pose.pose.orientation
        (_, _, self.odom_theta) = euler_from_quaternion([odom_q.x, odom_q.y, odom_q.z, odom_q.w])


    def run(self):

        while not rospy.is_shutdown():

            self.error_dist = math.sqrt(self.transformed_goal_pose.pose.position.x**2 + self.transformed_goal_pose.pose.position.y**2)
            self.error_ang = math.atan2(self.transformed_goal_pose.pose.position.y, self.transformed_goal_pose.pose.position.x)

            proportional_output_ang = self.Kp_ang * self.error_ang
            self.integral_error_ang += self.error_ang
            integral_output = self.Ki_ang * self.integral_error_ang
            
            total_output_ang = proportional_output_ang + integral_output

            proportional_output_dist = self.Kp_dist * self.error_dist
            self.integral_error_dist += self.error_dist
            integral_output_dist = self.Ki_dist * self.integral_error_dist

            total_output_dist = proportional_output_dist #+ integral_output_dist

            #print('error dist', self.error_dist)
            #print('error: ', self.error_ang)
            print(self.odom_theta - self.goal_theta)

            if abs(self.error_ang) > self.threshold_ang:
                self.twist.twist.angular.z = total_output_ang 
                # print("output = ", total_output_ang)
                self.twist.twist.linear.x = 0
                self.publisher_twist.publish(self.twist)

            elif self.error_dist > self.threshold_dist:
                # print('Correct angle')
                #self.angle_threshold = 0.5
                self.twist.twist.linear.x = total_output_dist
                self.twist.twist.angular.z = 0.0
                self.publisher_twist.publish(self.twist)

            elif self.odom_theta - self.goal_theta > 0.2:
                self.twist.twist.linear.x = 0.0
                self.twist.twist.angular.z = 0.2

            else:
                self.twist.twist.angular.z = 0
                self.twist.twist.linear.x = 0
                total_output_ang = 0
                total_output_dist = 0
                self.publisher_twist.publish(self.twist)


            self.publisher_twist.publish(self.twist)
            self.rate.sleep()



if __name__ == "__main__":
    try:
        rospy.init_node("move_to_goal") 
        move_to_goal()      
        rospy.spin()
    except rospy.ROSInterruptException:
        pass