#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Path

class give_path():

    def __init__(self):
        self.goal_publisher = rospy.Publisher('/goal', PoseStamped, queue_size=10)
        self.path_subscriber = rospy.Subscriber('/path_topic', Path, self.path_callback) 
        self.ready_for_pose_subscriber = rospy.Subscriber('/ready_for_pose', Bool, self.ready_for_path_callback)
        self.ready_for_pose_publisher = rospy.Publisher('/ready_for_pose', Bool, queue_size=1, latch=True)
        self.ready_for_new_path = rospy.Publisher('/ready_for_new_path', Bool, queue_size=1, latch=True)

        self.path = Path()
        self.ready_for_pose = Bool()
        self.ready_for_path = Bool()
        self.pose_to_send = 0

        self.main()

    def path_callback(self, msg):
        print('in path callback')
        self.path = msg


    def ready_for_path_callback(self, msg):
        self.ready_for_pose = msg.data

    def main(self):
        while not rospy.is_shutdown():
            if self.ready_for_pose and self.path.poses != []:
                print('ready!')
                self.ready_for_pose = False
                self.ready_for_pose_publisher.publish(self.ready_for_pose)
                self.goal_publisher.publish(self.path.poses[self.pose_to_send])
                self.pose_to_send = self.pose_to_send + 1
            
            if self.pose_to_send == len(self.path.poses):
                self.ready_for_path = True
                self.ready_for_new_path.publish(self.ready_for_path)
            else:
                self.ready_for_path = False
                self.ready_for_new_path.publish(self.ready_for_path)

                                        
                    


if __name__ == "__main__":
    try:
        rospy.init_node("give_path") 
        give_path()      
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
