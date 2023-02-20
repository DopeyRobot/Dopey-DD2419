import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point, TwistStamped, PoseStamped
from math import atan2


class planning():

    def __init__(self):    
        
        self.f = 10
        self.rate = rospy.Rate(self.f)
        self.transformedPose = PoseStamped()
        self.timeout = rospy.Duration(2)

        sub_odom = rospy.Subscriber("/motor/encoders", Odometry, self.get_odom)
        sub_goal = rospy.Subscriber('move_base_simple/goal', PoseStamped, self.get_goal) 
        self.pub_twist = rospy.Publisher("/motor_controller/twist", TwistStamped, queue_size = 1)

        self.x = 0.0
        self.y = 0.0 
        self.theta = 0.0

        self.run()


    def get_odom(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        self.rot_q = msg.pose.pose.orientation
        (roll, pitch, self.theta) = euler_from_quaternion([self.rot_q.x, self.rot_q.y, self.rot_q.z, self.rot_q.w])


    def get_goal(self, msg):

        self.goal = msg
        self.goal.x = msg.pose.pose.position.x
        self.goal.y = msg.pose.pose.position.y

    
    def run(self):

        speed = TwistStamped()

        while not rospy.is_shutdown():
            inc_x = self.goal.x -self.x
            inc_y = self.goal.y -self.y

            angle_to_goal = atan2(inc_y, inc_x)
            desired_heading = angle_to_goal - self.theta

        if abs(desired_heading) > 0.1:
            speed.linear.x = 0.0
            speed.angular.z = 0.3 * desired_heading / abs(desired_heading)
        else:
            speed.linear.x = 0.5
            speed.angular.z = 0.0

        self.pub_twist.publish(speed)
        self.rate.sleep()  

if __name__ == "__main__":
    try:
        rospy.init_node("planning_v2") 
        planning()      
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
        