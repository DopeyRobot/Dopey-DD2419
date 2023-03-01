#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import TransformStamped, PoseStamped
from aruco_msgs.msg import MarkerArray
import tf2_ros
from tf2_geometry_msgs import do_transform_pose

class aruco_dectection:
    def __init__(self):
        self.sub_goal = rospy.Subscriber('/aruco/markers', MarkerArray, self.aruco_callback)
        self.br = tf2_ros.TransformBroadcaster()
        self.buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.buffer)
        self.aruco_markers_frame = "camera_color_optical_frame"
        self.rate = rospy.Rate(10)
        self.parent = "map" #Old: "map"
        
        self.run()

    def aruco_callback(self, msg):
        transforms_list = []
        for detected_marker in msg.markers:

            if detected_marker.id == 500:
                
                detected_aruco_pose = PoseStamped()
                detected_aruco_pose.pose = detected_marker.pose.pose
                detected_aruco_pose.header.frame_id = self.aruco_markers_frame
                detected_aruco_pose.header.stamp = msg.header.stamp

                transform_is_possible = self.buffer.can_transform(self.parent, self.aruco_markers_frame, msg.header.stamp, rospy.Duration(2))
                if transform_is_possible:
                    # rospy.loginfo(f"Is the transform is possible?: {transform_is_possible}")
                    transformed_aruco_pose = self.buffer.transform(detected_aruco_pose, self.parent, rospy.Duration(2))

                    t = TransformStamped()
                    t.header.stamp = msg.header.stamp
                    t.header.frame_id = self.parent
                    t.child_frame_id = f"aruco/detected {detected_marker.id}"

                    # rospy.loginfo("Aruco marker detected with ID: %s" % str(detected_marker.id))

                    t.transform.translation.x = transformed_aruco_pose.pose.position.x
                    t.transform.translation.y = transformed_aruco_pose.pose.position.y
                    t.transform.translation.z = transformed_aruco_pose.pose.position.z

                    t.transform.rotation.x = transformed_aruco_pose.pose.orientation.x
                    t.transform.rotation.y = transformed_aruco_pose.pose.orientation.y
                    t.transform.rotation.z = transformed_aruco_pose.pose.orientation.z
                    t.transform.rotation.w = transformed_aruco_pose.pose.orientation.w

                    transforms_list.append(t)
                else:
                    rospy.loginfo("No transform found for the aruco marker")
                
        self.br.sendTransform(transforms_list)

        

    def run(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    rospy.init_node('display_marker500')
    aruco_dectection()
    rospy.spin()