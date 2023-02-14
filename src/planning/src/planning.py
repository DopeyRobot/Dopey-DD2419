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
        rospy.init_node("planning")
        self.publisher_goal = rospy.Publisher('motor_base_simple/goaLl', PoseStamped, queue_size=10)
        self.publisher_twist = rospy.Publisher('motor_controller/twist', Twist, queue_size=10)
        subscriber = rospy.Subscriber('motor_base_simple/goaLl', PoseStamped, self.callback)

        self.tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()

        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"

        point = Point()
        point.x = 1
        point.y = 1
        point.z = 0 

        pose_stamped.pose.position = point

        self.publisher_goal.publish(pose_stamped)

    def callback(self, pose_stamped):
        try:
            print('hello2')
            transform_is_possible = self.tf_buffer.can_transform("map", "base_link", pose_stamped.header.stamp, rospy.Duration(0.5))
            rospy.loginfo(f"Transform: {transform_is_possible}")
            transform = self.tf_buffer.lookup_transform("base_link", pose_stamped.header.frame_id, pose_stamped.header.stamp, rospy.Duration(0.5))
            print('hello3')
            transformed_position = tf2_geometry_msgs.do_transform_point(pose_stamped.pose.position, transform)
            transformed_orientation = tf2_geometry_msgs.do_transform_quaternion(pose_stamped.pose.orientation, transform)
            
            transformed_pose_stamped = PoseStamped()
            transformed_pose_stamped.header.stamp = pose_stamped.header.stamp
            transformed_pose_stamped.header.frame_id = "robot_frame"
            transformed_pose_stamped.pose.position = transformed_position.point
            
            self.publisher_goal.publish(transformed_pose_stamped)
            msg_twist = Twist() 
            msg_twist.angular.z = 4 * math.atan2(transformed_position.point.y, transformed_position.point.x)
            msg_twist.linear.x = 0.5 * math.sqrt(transformed_position.point.x ** 2 + transformed_position.point.y ** 2)

            self.publisher_twist.publish(msg_twist)
            print(msg_twist)
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Failed to transform pose: %s", e)

    def main(self):
        try:
            while not rospy.is_shutdown():
                rospy.spin()
        except rospy.ROSInterruptException:
            pass


if __name__ == "__main__":
    node = planning()        
    node.main()

    # move_base_simple/goal vill posestamped (koordinateran)
    # cmd_vel(motor_controller)
    # gör om min koordinat från map till robot frame
    # omvanlda punkten till robot_frame (lookup sen dotransform)
    # med mate beräkna hur man åker
    # publish twist linear.x och y