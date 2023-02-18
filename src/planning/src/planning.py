#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped
from robp_msgs.msg import Encoders
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point, TwistStamped, TransformStamped
import tf2_ros
import tf2_geometry_msgs
import math
#from rrt import path 

class planning():

    def __init__(self):

        self.Kp = 0.4
        self.Ki = 0.2
        self.Kd = 0.01
        self.integral_error = 0.0
        self.prev_error = 0.0
        self.angle_threshold = 0.1
        self.distance_threshold = 0.1


        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()

        
        self.f = 10
        self.rate = rospy.Rate(self.f)
        self.transformedPose = PoseStamped() #init PoseStamped message type
        self.targetframe = "base_link"
        self.currentframe = "map"
        self.timeout = rospy.Duration(2)


        self.publisher_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.publisher_twist = rospy.Publisher('motor_controller/twist', TwistStamped, queue_size=10)
        self.subscriber = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.callback) 


        self.run() 


    def callback(self, msg):

        ##INPUT MSG: POSE STAMPED
        #print(f"Check 2 : callback entered")
        detectedPoseStamped = PoseStamped()
        detectedPoseStamped.header.stamp = msg.header.stamp
        detectedPoseStamped.header.frame_id = msg.header.frame_id
        detectedPoseStamped.pose = msg.pose


        detectedTransformStamped = TransformStamped()


        #print("TimeStamp:", detectedPoseStamped.header.stamp)
        #print("Output CanTransform ",self.tf_buffer.can_transform(self.targetframe, detectedPoseStamped.header.frame_id, detectedPoseStamped.header.stamp, self.timeout))


        if self.tf_buffer.can_transform(self.targetframe, detectedPoseStamped.header.frame_id, detectedPoseStamped.header.stamp, self.timeout):


            #rospy.loginfo(f"Transform between target frame: {self.targetframe} and current frame: {self.currentframe} found")
            detectedTransformStamped = self.tf_buffer.lookup_transform(self.targetframe, self.currentframe, detectedPoseStamped.header.stamp, self.timeout)
            # tf2_geometry_msgs.do_transform_point(detectedPoseStamped, detectedTransformStamped)
            self.transformedPose = tf2_geometry_msgs.do_transform_pose(detectedPoseStamped, detectedTransformStamped)

            #print(f"Check 4: pose transformed")

        else:
            print('no transform')



    def get_new_nav_goal(self):
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = self.currentframe

        if self.transformedPose.pose.position.x**2 + self.transformedPose.pose.position.y**2 < self.distance_threshold**2:
            self.stop_robot()

        # point = Point() 
        # point.x = 1
        # point.y = 2
        # point.z = 0

        # pose_stamped.pose.position = point
        # self.publisher_goal.publish(pose_stamped)

        return pose_stamped


    def publish_twist(self):

        twist = TwistStamped()  
        twist.header.stamp = self.transformedPose.header.stamp 
        twist.header.frame_id = self.transformedPose.header.frame_id 

        # calculate error between current position and desired position
        error = math.atan2(self.transformedPose.pose.position.y, self.transformedPose.pose.position.x)

        # if the angle is greater than 0.1 radians, rotate in place
        '''if abs(error) > self.angle_threshold:
            print('true')
            twist.twist.angular.z = 0.5
            twist.twist.linear.x = 0.0'''

        
        # calculate proportional control output
        proportional_output = self.Kp * error

        # calculate integral control output
        self.integral_error += error
        integral_output = self.Ki * self.integral_error

        print(self.integral_error)

        # calculate derivative control output
        # derivative_error = error - self.prev_error
        # derivative_output = self.Kd * derivative_error

        # calculate total control output (proportional + integral + derivative)
        total_output = proportional_output 

        # update previous error for next iteration
        self.prev_error = error

        # publish to twist
        twist.twist.angular.z = total_output
        twist.twist.linear.x = 0.05 * math.sqrt(self.transformedPose.pose.position.x**2 + self.transformedPose.pose.position.y**2)
        
        self.publisher_twist.publish(twist)

    def stop_robot(self):

        twist = TwistStamped()
        twist.header.stamp = rospy.Time.now()
        twist.header.frame_id = self.currentframe
        twist.twist.angular.z = 0.0
        twist.twist.linear.x = 0.0
        self.publisher_twist.publish(twist)


    def run(self):

        while not rospy.is_shutdown():
            pose_stamped = self.get_new_nav_goal()
            self.publisher_goal.publish(pose_stamped)
            self.publish_twist()

            self.rate.sleep()


if __name__ == "__main__":
    try:
        rospy.init_node("planning") 
        planning()      
        rospy.spin()
    except rospy.ROSInterruptException:
        pass






'''
    def run(self):
        # Put in run file, because later when obstacles are introduced, the new position to be navigated to will be updated continuously
        # Publishing is done post transformation from map into base frame 
        while not rospy.is_shutdown():
            
            ##PUBLISH NAV GOAL COORDINATE
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = self.currentframe

            ##SECTION NEEDS TO BE REPLACED WITH PATH PLANNING FUNCTION WITH PUBLISHES ONTO MOVE_BASE_SIMPLE/GOAL
            # point = Point() 
            # point.x = 1
            # point.y = 2
            # point.z = 0

            # pose_stamped.pose.position = point
            # self.publisher_goal.publish(pose_stamped)

            ##PUBLISH TWIST SECTION


            twist = TwistStamped()  
            twist.header.stamp = self.transformedPose.header.stamp 
            twist.header.frame_id = self.transformedPose.header.frame_id 

            # calculate error between current position and desired position
            error = math.atan2(self.transformedPose.pose.position.y, self.transformedPose.pose.position.x)

            # calculate proportional control output
            proportional_output = self.Kp * error

            # calculate integral control output
            self.integral_error += error
            integral_output = self.Ki * self.integral_error

            # calculate total control output (proportional + integral)
            total_output = proportional_output + integral_output

            # update previous error for next iteration
            self.prev_error = error

            # publish to twist
            twist.twist.angular.z = total_output
            twist.twist.linear.x = 0.05 * math.sqrt(self.transformedPose.pose.position.x**2 + self.transformedPose.pose.position.y**2)
            self.publisher_twist.publish(twist)

            #print("Check 5: published twist")
            self.rate.sleep()

            

if __name__ == "__main__":
    try:
        rospy.init_node("planning") 
        planning()      
        rospy.spin()
    except rospy.ROSInterruptException:
        pass '''