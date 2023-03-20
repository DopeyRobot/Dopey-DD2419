#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Path

class give_path():

    def __init__(self):
        print('in init')
        self.goal_publisher = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.path_subscriber = rospy.Subscriber('/path_topic', Path, self.path_callback) 
        self.ready_for_path_subscriber= rospy.Subscriber('/ready_for_pose', Bool, self.ready_for_path_callback)

        self.path = Path()
        self.ready_for_path = False

        self.main()

    def path_callback(self, msg):
        self.path = msg

    def ready_for_path_callback(self, msg):
        self.ready_for_path = msg.data

    def main(self):
        
        while not rospy.is_shutdown():
            for pose in self.path.poses:
                if self.ready_for_path:
                    print('ready')
                    self.goal_publisher.publish(pose)


if __name__ == "__main__":
    try:
        rospy.init_node("give_path") 
        give_path()      
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
