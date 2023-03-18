#!/usr/bin/env python

import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from geometry_msgs.msg import PoseStamped
from rrt import RRT

class RRTMoveToGoal:
    def __init__(self):
        rospy.init_node('execute')
        
        # Initialize the action client for move_to_goal
        self.client = SimpleActionClient('move_to_goal', MoveBaseAction)
        self.client.wait_for_server()

        # Create an instance of the RRT class
        self.rrt_planner = RRT()

        # Create a subscriber to receive goal locations
        rospy.Subscriber('/goal', PoseStamped, self.goal_callback)

    def goal_callback(self, goal):
        # Generate path using RRT path planner
        path = self.rrt_planner.plan_path(current_pos, goal.pose.position)

        # Send path to move_to_goal action server
        for pose in path:
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = 'map'
            goal.target_pose.pose.position.x = pose[0]
            goal.target_pose.pose.position.y = pose[1]
            goal.target_pose.pose.orientation.w = pose[2]
            self.client.send_goal(goal)
            self.client.wait_for_result()
        
if __name__ == '__main__':
    rrt_move_to_goal = RRTMoveToGoal()
    rospy.spin()
