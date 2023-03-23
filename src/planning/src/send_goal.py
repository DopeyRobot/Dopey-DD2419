#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
import numpy as np

class send_goal():

    def __init__(self):

        self.publisher_goal = rospy.Publisher('/send_goal', PoseStamped, queue_size=10)

        self.goal = PoseStamped()
        self.goal_pose_x = np.random.random()
        self.goal_pose_y = np.random.random()

        self.run()

    
    def run(self):

        while not rospy.is_shutdown():

            self.goal.pose.position.x = self.goal_pose_x
            self.goal.pose.position.y = self.goal_pose_y

            self.publisher_goal.publish(self.goal)


if __name__ == "__main__":
    try:
        rospy.init_node("send_goal") 
        send_goal()      
        rospy.spin()
    except rospy.ROSInterruptException:
        pass