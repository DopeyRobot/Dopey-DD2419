#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Path
import py_trees as pt

class give_path():

    def __init__(self):
        self.goal_pub = rospy.Publisher('/goal', PoseStamped, queue_size=10)
        self.path_sub = rospy.Subscriber('/path_topic', Path, self.path_callback) 
        self.ready_for_pose_sub = rospy.Subscriber('/ready_for_pose', Bool, self.ready_for_path_callback)
        self.ready_for_pose_pub = rospy.Publisher('/ready_for_pose', Bool, queue_size=1, latch=True)
        self.ready_for_new_path = rospy.Publisher('/ready_for_new_path', Bool, queue_size=1, latch=True)

        self.path = Path()
        self.ready_for_pose = Bool()
        self.ready_for_path = Bool()
        self.pose_to_send = 0

        self.main()

    def path_callback(self, msg):
        self.path = msg

    def ready_for_path_callback(self, msg):
        self.ready_for_pose = msg.data

    def update(self):

            if self.ready_for_pose and self.path.poses != []:
                print('Ready for new pose!')
                self.ready_for_pose = False
                self.ready_for_pose_pub.publish(self.ready_for_pose)
                self.goal_pub.publish(self.path.poses[self.pose_to_send])
                self.pose_to_send = self.pose_to_send + 1

                #Send RUNNING to behaviour tree
                return pt.common.Status.RUNNING
            
            if self.pose_to_send == len(self.path.poses):
                
                #Kalla på function som kollar vart man är och om det är rätt skicka success.

                self.pose_to_send = 0
                self.ready_for_path = True
                self.ready_for_new_path.publish(self.ready_for_path)

                #Should probably implement a comparison between where base_link is in tf_tree and last desired pose
                #If base_link is close enough to last desired pose, send SUCCESS to behaviour tree
                    #Send SUCCESS to behaviour tree
                return pt.common.Status.SUCCESS
            
                #Else send FAILURE to behaviour tree
                # return pt.common.Status.FAILURE

            else:
                self.ready_for_path = False
                self.ready_for_new_path.publish(self.ready_for_path)