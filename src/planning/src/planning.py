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

        self.Kp = 0.0007
        self.Ki = 0.002
        self.Kd = 0.03
        self.integral_error = 0.0
        self.integral_error_dist = 0.0
        self.prev_error = 0.0
        self.angle_threshold = 0.1
        self.distance_threshold = 0.1
        self.first_rot = False
        self.first_lin = False
        self.error = 0.0
        self.detectedPoseStamped = PoseStamped()
        self.detectedTransformStamped = TransformStamped()

        ## Max linear velocity (m/s)
        self.max_linear_velocity = 0.05
        ## Max angular velocity (rad/s)
        self.max_angular_velocity = 0.05

        #print('Check 1: init file entered')
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.br = tf2_ros.TransformBroadcaster()

        self.f = 4
        self.rate = rospy.Rate(self.f)
        self.transformedPose = PoseStamped() #init PoseStamped message type
        self.targetframe = "base_link"
        self.currentframe = "odom"
        self.timeout = rospy.Duration(2)

        self.publisher_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.publisher_twist = rospy.Publisher('motor_controller/twist', TwistStamped, queue_size=10)
        self.subscriber = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.callback) 

        self.run() #calls run function which will continuously in a loop publish newest point then subscribe as well as extract info for new point



    def callback(self, msg):
        ##INPUT MSG: POSE STAMPED
        #print(f"Check 2 : callback entered")
        self.detectedPoseStamped.header.stamp = msg.header.stamp
        self.detectedPoseStamped.header.frame_id = msg.header.frame_id
        self.detectedPoseStamped.pose = msg.pose

        if self.tf_buffer.can_transform(self.targetframe, self.detectedPoseStamped.header.frame_id, self.detectedPoseStamped.header.stamp, self.timeout):
            #print(f"Check 3 : transform found")

            #rospy.loginfo(f"Transform between target frame: {self.targetframe} and current frame: {self.currentframe} found")
            self.detectedTransformStamped = self.tf_buffer.lookup_transform(self.targetframe, self.currentframe, self.detectedPoseStamped.header.stamp, self.timeout)
            self.transformedPose = tf2_geometry_msgs.do_transform_pose(self.detectedPoseStamped, self.detectedTransformStamped)

            #print(f"Check 4: pose transformed")

        #Sprint("TimeStamp:", self.detectedPoseStamped.header.stamp)
        #print("Output CanTransform ",self.tf_buffer.can_transform(self.targetframe, self.detectedPoseStamped.header.frame_id, self.detectedPoseStamped.header.stamp, self.timeout))



    def run(self):
        # Put in run file, because later when obstacles are introduced, the new position to be navigated to will be updated continuously
        # Publishing is done post transformation from map into base frame 
        while not rospy.is_shutdown():

            ##PUBLISH TWIST SECTION
            twist = TwistStamped()  
            twist.header.stamp = self.transformedPose.header.stamp 
            twist.header.frame_id = self.transformedPose.header.frame_id 

            error_dist = math.sqrt(self.transformedPose.pose.position.x**2 + self.transformedPose.pose.position.y**2)
            self.error = math.atan2(self.transformedPose.pose.position.y, self.transformedPose.pose.position.x)
            #print('error dist', error_dist)

            proportional_output = self.Kp * self.error
            proportional_output_dist = self.Kp * error_dist

            self.integral_error += self.error
            integral_output = self.Ki * self.integral_error

            self.integral_error_dist += error_dist
            integral_output_dist = self.Ki * self.integral_error_dist

            total_output = proportional_output + integral_output
            total_output_dist = proportional_output_dist + integral_output_dist
            print('error: ', abs(self.error))

            if abs(self.error) > self.angle_threshold:
                twist.twist.angular.z = total_output 
                twist.twist.linear.x = total_output_dist
                if twist.twist.angular.z > self.max_angular_velocity:
                    total_output = self.max_angular_velocity
                if twist.twist.linear.x > self.max_linear_velocity:
                    total_output_dist = self.max_linear_velocity
                    
            else:
                print('Correct angle')
                twist.twist.linear.x = 0.0
                twist.twist.angular.z = 0.0
                rospy.sleep(10)
       
            self.publisher_twist.publish(twist)
            self.rate.sleep()



if __name__ == "__main__":
    try:
        rospy.init_node("planning") 
        planning()      
        rospy.spin()
    except rospy.ROSInterruptException:
        pass