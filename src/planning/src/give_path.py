#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
from nav_msgs.msg import Path
import tf2_geometry_msgs

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
    
    def get_current_pose(self):    

        robot_pose = PoseStamped()
        robot_pose.header.stamp = rospy.Time.now()
        base_link_origin = PoseStamped()
        base_link_origin.header.stamp = robot_pose.header.stamp

        transform_to_map = self.buffer.lookup_transform("base_link", "map", robot_pose.header.stamp , rospy.Duration(1))  
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

    def main(self):
        while not rospy.is_shutdown():

            if self.ready_for_pose and self.path.poses != []:
                print('Ready for new pose!')
                self.ready_for_pose = False
                self.ready_for_pose_pub.publish(self.ready_for_pose)
                self.goal_pub.publish(self.path.poses[self.pose_to_send])
                self.pose_to_send = self.pose_to_send + 1
                #Skicka running
            
            if self.pose_to_send == len(self.path.poses):
                
                #Kalla på function som kollar vart man är och om det är rätt skicka success.

                self.pose_to_send = 0
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



