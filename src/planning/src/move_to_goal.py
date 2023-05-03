#!/usr/bin/env python3
import rospy
import math
import numpy as np
import tf_conversions
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Twist
from geometry_msgs.msg import  Twist
from tf2_geometry_msgs import PoseStamped 
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf.transformations import euler_from_quaternion
from tf2_ros import Buffer, TransformListener, TransformStamped, TransformBroadcaster

class PID:
    def __init__(self, P:float = 0.0, I:float = 0.0, D:float=0.0) -> None:
        self.P = P
        self.I = I
        self.D = D

        self.prev_error = 0.0
        self.int_error = 0.0
        self.dt = 0.1

    def __call__(self, error):
        P_err = error
        D_err = P_err - self.prev_error
        I_err = self.int_error

        command = self.P*P_err + self.I*I_err + self.D*D_err
        self.prev_error = P_err
        self.int_error += P_err * self.dt

        return command


class move_to_goal():

    def __init__(self):
        self.ang1PID = PID(1e-1, 0.0, 0.0)
        self.distPID = PID(3e-1, 0.0, 0.0)
        self.ang2PID = PID(1e-1, 0.0, 0.0)

        self.threshold_ang1 = 0.05
        self.threshold_ang2 = 0.05
        self.threshold_dist = 0.05
 
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
        self.currentframe = "map"
        self.timeout = rospy.Duration(2)


        rospy.sleep(2)

        self.publisher_twist = rospy.Publisher('motor_controller/twist', Twist, queue_size=10)
        self.ready_for_pose_publisher = rospy.Publisher('/ready_for_pose', Bool, queue_size=1, latch=True)
        self.goal_subscriber = rospy.Subscriber('/goal', PoseStamped, self.goal_callback) 
        self.odom_subscriber = rospy.Subscriber('/odometry', Odometry, self.odom_callback) 

        #self.ready_for_pose.data = True # Maybe not needed?
        #self.ready_for_pose_publisher.publish(self.ready_for_pose)

        self.arrived2point = False
        self.run() 


    def goal_callback(self, msg):
        # rospy.loginfo("Recived new goal")

        # self.ready_for_pose.data = False #TODO
        # self.ready_for_pose_publisher.publish(self.ready_for_pose)
        self.goal_pose = PoseStamped()
        self.goal_pose.pose = msg.pose
        self.goal_pose.header.stamp = msg.header.stamp
        self.goal_pose.header.frame_id = msg.header.frame_id

        # print(self.goal_pose.pose)

        self.arrived2point = False

    def get_current_pose(self):    

        robot_pose = PoseStamped()
        robot_pose.header.stamp = rospy.Time.now()
        base_link_origin = PoseStamped()
        base_link_origin.header.stamp = robot_pose.header.stamp

        transform_to_map = self.tf_buffer.lookup_transform("map", "base_link", robot_pose.header.stamp , rospy.Duration(1))  #TODO: what was workign befroe was reverse order of target/soruce frame
        baseInMapPose = tf2_geometry_msgs.do_transform_pose(base_link_origin, transform_to_map)

        robot_pose.pose.position.z = baseInMapPose.pose.position.z
        robot_pose.pose.position.x = baseInMapPose.pose.position.x
        robot_pose.pose.position.y = baseInMapPose.pose.position.y
        robot_pose.pose.orientation.w = baseInMapPose.pose.orientation.w
        robot_pose.pose.orientation.x = baseInMapPose.pose.orientation.x
        robot_pose.pose.orientation.y = baseInMapPose.pose.orientation.y
        robot_pose.pose.orientation.z = baseInMapPose.pose.orientation.z

        robot_pose.header.frame_id = "map"
        
        return robot_pose

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
                (_, _, self.goal_theta) = euler_from_quaternion([rot_q.x, rot_q.x, rot_q.y, rot_q.z])
                #(_, _, self.goal_theta) = euler_from_quaternion([rot_q.w, rot_q.x, rot_q.y, rot_q.z])
                # (_, _, self.goal_theta) = tf_conversions.transformations.euler_from_quaternion([rot_q.w, rot_q.x, rot_q.y, rot_q.z])

                error_dist = math.sqrt(self.transformed_goal_pose.pose.position.x**2 + self.transformed_goal_pose.pose.position.y**2)
                error_ang1 = math.atan2(self.transformed_goal_pose.pose.position.y, self.transformed_goal_pose.pose.position.x)

                current_angel = self.get_current_pose().pose.orientation.w
                error_ang2 = current_angel - self.goal_theta
                
                ang1out = self.ang1PID(error_ang1)
                distout = self.distPID(error_dist)
                ang2out = self.ang2PID(error_ang2)
            
                if not self.arrived2point:
                    
                    rospy.logdebug('Adjusting ang1')
                    rospy.logdebug(error_ang1)
                    self.twist.angular.z = ang1out
    
                    rospy.logdebug("Adjusting dist")
                    rospy.logdebug(error_dist)
                    self.twist.linear.x = distout*np.exp(-np.abs(error_ang1)*10)
                    
                    if error_dist < self.threshold_dist and abs(error_ang1) < self.threshold_ang1:
                        rospy.logdebug("Correct pose!")
                        self.twist.angular.z = 0
                        self.twist.linear.x = 0
                        ang1out = 0
                        distout = 0
                        self.arrived2point = True
                        
                else:
                    if abs(error_ang2) > self.threshold_ang2:
                        rospy.logdebug("Ajusting ang2")
                        rospy.logdebug(error_ang2)
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = ang2out

                    else:
                        rospy.logdebug("Done")
                        self.twist.linear.x = 0.0
                        self.twist.angular.z = 0.0
                        self.ready_for_pose.data = True
                        self.ready_for_pose_publisher.publish(self.ready_for_pose)
                        rospy.sleep(3)


                self.publisher_twist.publish(self.twist)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node("move_to_goal")#,log_level=rospy.DEBUG)
        move_to_goal()      
        rospy.spin()
    except rospy.ROSInterruptException:
        pass