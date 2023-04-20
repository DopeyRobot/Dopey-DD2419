#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped

class send_goal():

    def __init__(self):

        self.publisher_goal = rospy.Publisher('/send_goal', PoseStamped, queue_size=1)

        self.goal = PoseStamped()
        self.goal_pose_x = np.random.random()*2
        self.goal_pose_y = np.random.random()*2

        self.run()
    
    def run(self):
        while not rospy.is_shutdown():
            self.goal.pose.position.x = self.goal_pose_x
            self.goal.pose.position.y = self.goal_pose_y
            self.goal.header.frame_id = "map"
            self.publisher_goal.publish(self.goal)
            print("published")


if __name__ == "__main__":
    # try:
    rospy.init_node("send_goal") 
    send_goal()      
    rospy.spin()
    # except rospy.ROSInterruptException:
    #     pass