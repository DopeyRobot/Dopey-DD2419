#!/usr/bin/env python3
import rospy
import math
import numpy as np
import tf2_geometry_msgs
from geometry_msgs.msg import PoseStamped, Twist
from geometry_msgs.msg import  Twist
from tf2_geometry_msgs import PoseStamped 
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String
from std_srvs.srv import EmptyResponse
from tf.transformations import euler_from_quaternion
from tf2_ros import Buffer, TransformListener, TransformStamped, TransformBroadcaster
from planning.srv import lastAngle, lastAngleRequest, lastAngleResponse
from bounding_box_detection.srv import twoStrInPoseOut, twoStrInPoseOutRequest, closestObj, closestObjRequest

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

    def __init__(self,runInit=True):
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

        self.f = 30
        self.rate = rospy.Rate(self.f)
        self.targetframe = "base_link"
        self.currentframe = "map"
        self.timeout = rospy.Duration(2)

        self.lastang = None

        rospy.sleep(2)
       
        self.publisher_twist = rospy.Publisher('motor_controller/twist', Twist, queue_size=1)
        self.ready_for_pose_publisher = rospy.Publisher('/ready_for_pose', Bool, queue_size=1, latch=True)
        self.pub_goal_approach = rospy.Publisher('/goal', PoseStamped, queue_size=10)
        self.approach_bool_publisher = rospy.Publisher('/approach_done', Bool, queue_size=1, latch=True)

        self.goal_subscriber = rospy.Subscriber('/goal', PoseStamped, self.goal_callback) 

        self.lastAngle_srv = None
        self.getframe_client = rospy.ServiceProxy("/get_object_pose", twoStrInPoseOut)

        self.arrived2point = False
        if runInit:
            self.run() 


    def lastAngle_cb(self, req:lastAngleRequest):
        #angles are dealt with in base link reference frame

        current_ob = req.goal_frameid #get string data and make into rospy String
        req0 = twoStrInPoseOutRequest()
        req0.str1 = String(self.targetframe) #baselink
        req0.str2 = current_ob
        object_pose = self.getframe_client(req0)

        gx = object_pose.pose.pose.position.x
        gy = object_pose.pose.pose.position.y

        anglePID = PID(0.1, 0, 0)
        distancePID = PID(0.1, 0, 0)

        error_ang = math.atan2(gy, gx) 
        error_dist = np.sqrt(gx**2 + gy**2)
        
        dist_cont = distancePID(error_dist)
        angle_cont = anglePID(error_ang)
        dist_angle_cont = dist_cont*np.exp(-np.abs(error_ang)*10)
        
        twist_msg = Twist()
        approach_bool = Bool()
        approach_bool.data = False

        while np.abs(error_ang) > 0.05 or np.abs(error_dist) > 0.10:
        
            req1 = twoStrInPoseOutRequest()
            req1.str1 = String(self.targetframe) #baselink
            req1.str2 = current_ob 
            object_pose_new = self.getframe_client(req1)

            gx = object_pose_new.pose.pose.position.x
            gy = object_pose_new.pose.pose.position.y

            new_goal = PoseStamped()
            new_goal.pose.position.x = gx
            new_goal.pose.position.y = gy

            self.goal_pose = None

            # self.pub_goal_approach.publish(new_goal)

            error_ang_new = math.atan2(gy, gx) 
            angle_cont = anglePID(error_ang_new)

            error_dist_new = np.sqrt(gx**2 + gy**2)
            dist_cont = distancePID(error_dist_new)

            dist_angle_cont_new = dist_cont*np.exp(-np.abs(error_ang_new)*10)

            twist_msg.angular.z = angle_cont
            twist_msg.linear.x = dist_angle_cont
            self.publisher_twist.publish(twist_msg) #input new twist message

            error_ang = error_ang_new
            dist_angle_cont = dist_angle_cont_new
            error_dist= error_dist_new
            self.approach_bool_publisher.publish(approach_bool)
            if np.abs(error_dist) < 0.14:
                break

        approach_bool.data = True
        twist_msg.angular.z = 0
        twist_msg.linear.x = 0
   
        self.publisher_twist.publish(twist_msg)
        self.approach_bool_publisher.publish(approach_bool)
        print("lastAngle service done")

        return EmptyResponse()

    def goal_callback(self, msg):
        self.goal_pose = PoseStamped()
        self.goal_pose.pose = msg.pose
        self.goal_pose.header.stamp = msg.header.stamp
        self.goal_pose.header.frame_id = msg.header.frame_id

        self.arrived2point = False

    def run(self):
        self.lastAngle_srv = rospy.Service("/lastAngle", lastAngle, self.lastAngle_cb) 
        while not rospy.is_shutdown():

            if self.goal_pose:
                
                self.goal_pose.header.frame_id = self.currentframe
                self.goal_pose.header.stamp = rospy.Time.now()
                self.transformed_goal_pose = self.tf_buffer.transform(self.goal_pose, self.targetframe, self.timeout)

                goal_q = self.goal_pose.pose.orientation
                (_, _, self.goal_theta) = euler_from_quaternion([goal_q.w, goal_q.x, goal_q.y, goal_q.z])


                error_dist = math.sqrt(self.transformed_goal_pose.pose.position.x**2 + self.transformed_goal_pose.pose.position.y**2)
                error_ang1 = math.atan2(self.transformed_goal_pose.pose.position.y, self.transformed_goal_pose.pose.position.x) #self.get_err_a1()#
                self.lastang = error_ang1
                
                ang1out = self.ang1PID(error_ang1)
                distout = self.distPID(error_dist)

            
                if not self.arrived2point:
                    rospy.logdebug('Adjusting ang1')
                    self.twist.angular.z = ang1out
    
                    rospy.logdebug("Adjusting dist")
                    self.twist.linear.x = distout*np.exp(-np.abs(error_ang1)*10)
                    
                    if error_dist < self.threshold_dist:
                        rospy.logdebug("Correct pose!")
                        ang1out = 0
                        distout = 0
                        self.arrived2point = True
                        
                else:
                        rospy.logdebug("Done")
                        self.ready_for_pose.data = True
                        self.ready_for_pose_publisher.publish(self.ready_for_pose)
        
                        self.__init__(runInit=False)


                self.publisher_twist.publish(self.twist)
            self.rate.sleep()

if __name__ == "__main__":
    try:
        rospy.init_node("move_to_goal")#,log_level=rospy.DEBUG)
        move_to_goal()      
        rospy.spin()
    except rospy.ROSInterruptException:
        pass