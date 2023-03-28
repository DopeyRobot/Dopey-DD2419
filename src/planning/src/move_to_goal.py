#!/usr/bin/env python3
import rospy
import math
from tf2_ros import Buffer, TransformListener, TransformStamped, TransformBroadcaster
import tf_conversions
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Twist
from geometry_msgs.msg import  Twist
from tf2_geometry_msgs import PoseStamped 
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion

class move_to_goal():

    def __init__(self):

        self.Kp_ang1 = 0.05
        self.Ki_ang1 = 0.0002
        self.Kd_ang1 = 1.0
        self.Kp_dist = 0.0008
        self.Ki_dist = 0.0002
        self.Kd_dist = 0.5
        self.Kp_ang2 = 0.003
        self.Ki_ang2 = 0.00001
        self.Kd_ang2 = 1.0

        self.integral_error_ang1 = 0.0
        self.integral_error_dist = 0.0
        self.integral_error_ang2 = 0.0
        self.prev_error_ang1 = 0.0
        self.prev_error_dist = 0.0
        self.prev_error_ang2 = 0.0

        self.threshold_ang1 = 0.15
        self.threshold_ang2 = 0.15
        self.threshold_dist = 0.15
 
        self.goal_theta = 0.0
        self.odom_theta = 0.0
        self.goal_pose = None
        self.transformed_goal_pose = None 

        self.twist = Twist()  
        self.odom = Odometry()
        self.ready_for_pose = Bool()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer)
        self.br = TransformBroadcaster()

        self.f = 50
        self.rate = rospy.Rate(self.f)
        self.targetframe = "base_link"
        self.currentframe = "odom"
        self.timeout = rospy.Duration(2)


        rospy.sleep(2)

        self.publisher_twist = rospy.Publisher('motor_controller/twist', Twist, queue_size=10)
        self.ready_for_pose_publisher = rospy.Publisher('/ready_for_pose', Bool, queue_size=1, latch=True)
        self.goal_subscriber = rospy.Subscriber('/goal', PoseStamped, self.goal_callback) 
        self.odom_subscriber = rospy.Subscriber('/odometry', Odometry, self.odom_callback) 

        self.ready_for_pose.data = True # Maybe not needed?
        self.ready_for_pose_publisher.publish(self.ready_for_pose)

        self.arrived2point = False
        self.run() 


    def goal_callback(self, msg):
        rospy.loginfo("Recived new goal")

        self.ready_for_pose.data = False
        self.ready_for_pose_publisher.publish(self.ready_for_pose)
        self.goal_pose = PoseStamped()
        self.goal_pose.pose = msg.pose
        self.goal_pose.header.stamp = msg.header.stamp
        self.goal_pose.header.frame_id = msg.header.frame_id

        print(self.goal_pose.pose)

        self.arrived2point = False
        

    def odom_callback(self, msg):
        self.odom = msg
        odom_q = self.odom.pose.pose.orientation
        (_, _, self.odom_theta) = tf_conversions.transformations.euler_from_quaternion([odom_q.w, odom_q.x, odom_q.y, odom_q.z])



        


    def run(self):

        while not rospy.is_shutdown():

            if self.goal_pose:

                self.goal_pose.header.frame_id = self.currentframe
                self.goal_pose.header.stamp = rospy.Time.now()
                self.transformed_goal_pose = self.tf_buffer.transform(self.goal_pose, self.targetframe, self.timeout)

                rot_q = self.goal_pose.pose.orientation
                (_, _, self.goal_theta) = euler_from_quaternion([rot_q.w, rot_q.x, rot_q.y, rot_q.z])
                # (_, _, self.goal_theta) = tf_conversions.transformations.euler_from_quaternion([rot_q.w, rot_q.x, rot_q.y, rot_q.z])

                error_dist = math.sqrt(self.transformed_goal_pose.pose.position.x**2 + self.transformed_goal_pose.pose.position.y**2)
                error_ang1 = math.atan2(self.transformed_goal_pose.pose.position.y, self.transformed_goal_pose.pose.position.x)
                current_angel = 0.0
                error_ang2 = current_angel - self.goal_theta
                
                proportional_output_ang1 = self.Kp_ang1 * error_ang1
                self.integral_error_ang1 += error_ang1
                self.integral_output_ang1 = self.Ki_ang1 * self.integral_error_ang1
                derivative_error_ang1 = error_ang1 - self.prev_error_ang1
                self.prev_error_ang1 = error_ang1
                derivative_output_ang1 = self.Kd_ang1 * derivative_error_ang1
                total_output_ang1 = proportional_output_ang1 + self.integral_output_ang1 + derivative_output_ang1

                proportional_output_dist = self.Kp_dist * error_dist
                self.integral_error_dist += error_dist
                integral_output_dist = self.Ki_dist * self.integral_error_dist
                derivative_error_dist = error_dist - self.prev_error_dist
                self.prev_error_dist = error_dist
                derivative_output_dist = self.Kd_dist * derivative_error_dist
                total_output_dist = proportional_output_dist + integral_output_dist + derivative_output_dist

                proportional_output_ang2 = self.Kp_ang2 * error_ang2
                self.integral_error_ang2 += error_ang2
                integral_output_ang2 = self.Ki_ang2 * self.integral_error_ang2
                derivative_error_ang2 = error_ang2 - self.prev_error_ang2
                self.prev_error_ang2 = error_ang2
                derivative_output_ang2 = self.Kd_ang2 * derivative_error_ang2
                total_output_ang2 = proportional_output_ang2 + integral_output_ang2 + derivative_output_ang2

                if total_output_ang1 > 0.5:
                    total_output_ang1 =0.5
            
                if not self.arrived2point:
                    
                    if abs(error_ang1) > self.threshold_ang1:
                        print('Adjusting ang1',error_ang1, total_output_ang1)
                        self.twist.angular.z = total_output_ang1
                        self.twist.linear.x = 0.0
        
                    elif error_dist > self.threshold_dist:
                        print("Correct angle! Adjusting dist", error_dist)
                        self.twist.linear.x = total_output_dist
                        self.twist.angular.z = 0.0

                    else:
                        rospy.loginfo("Correct pose!")
                        self.twist.angular.z = 0
                        self.twist.linear.x = 0
                        total_output_ang1 = 0
                        total_output_dist = 0
                        self.arrived2point = True
                else:
                    if abs(error_ang2) > self.threshold_ang2:
                        print("Ajusting ang2", error_ang2)
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = total_output_ang2

                    else:
                        rospy.loginfo("Done")
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = 0.0

                        self.ready_for_pose.data = True
                        self.ready_for_pose_publisher.publish(self.ready_for_pose)


                self.publisher_twist.publish(self.twist)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node("move_to_goal") 
        move_to_goal()      
        rospy.spin()
    except rospy.ROSInterruptException:
        pass