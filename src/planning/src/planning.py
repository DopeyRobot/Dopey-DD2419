#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, Twist
import tf2_ros
import tf2_geometry_msgs
import math
#from rrt import path 

class planning():

    def __init__(self):

        print('hello1')
        
        self.rate = rospy.Rate(10)

        self.publisher_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.publisher_twist = rospy.Publisher('motor_controller/twist', Twist, queue_size=10)
        self.subscriber = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        
        self.point = Point()
        self.point.x = 1
        self.point.y = 1
        self.point.z = 0 

        pose_stamped.pose.position = self.point
        self.publisher_goal.publish(pose_stamped)
        rospy.loginfo(pose_stamped)

    def callback(self, pose_stamped):
        print(pose_stamped)
        
        try:
            print('hello2')
            transform_is_possible = self.tf_buffer.can_transform("map", "base_link", pose_stamped.header.stamp, rospy.Duration(0.5))
            rospy.loginfo(f"Transform: {transform_is_possible}")
            print('hello3')
            transform = self.tf_buffer.lookup_transform("map", "base_link", rospy.Time.now(), rospy.Duration(1.0))
    
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped,transform)
    
            transformed_position = transformed_pose.pose.position
            transformed_orientation = transformed_pose.pose.orientation

            transformed_pose_stamped = PoseStamped()
            transformed_pose_stamped.header.stamp = pose_stamped.header.stamp
            transformed_pose_stamped.header.frame_id = "base_frame"
            transformed_pose_stamped.pose.position = transformed_orientation
       
            self.publisher_goal.publish(transformed_pose_stamped)

            msg_twist = Twist()
            msg_twist.angular.z = 4 * math.atan2(transformed_position.y, transformed_position.x)
            msg_twist.linear.x = 0.5 * math.sqrt(transformed_position.x ** 2 + transformed_position.y ** 2)

            self.publisher_twist.publish(msg_twist)
    

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Failed to transform pose: %s", e)


    def main(self):
        while not rospy.is_shutdown():
            self.rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("planning") 
        planning()      
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

    # move_base_simple/goal vill posestamped (koordinateran)
    # cmd_vel(motor_controller)
    # gör om min koordinat från map till robot frame
    # omvanlda punkten till robot_frame (lookup sen dotransform)
    # med mate beräkna hur man åker
    # publish twist linear.x och y