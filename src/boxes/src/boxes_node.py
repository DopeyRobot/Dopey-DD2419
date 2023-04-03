#!/usr/bin/env python3
import rospy
from typing import List
from tf2_geometry_msgs import PoseStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, TransformStamped
from sensor_msgs.msg import LaserScan
from bounding_box_detection.srv import instanceNames


class BoxNode:
    def __init__(self) -> None:
        self.laser_scan_sub = rospy.Subscriber(
            "/scan", LaserScan, self.laser_scan_callback
        )
        self.scan: LaserScan = rospy.wait_for_message("/scan", LaserScan, timeout=5.0)
        self.reference_frame = "map"
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer)
        self.broadcaster = TransformBroadcaster()
        self.memory_list_srv = rospy.ServiceProxy("/instances_names", instanceNames)

    def laser_scan_callback(self, msg: LaserScan) -> None:
        self.scan = msg

    def get_boxes(self) -> List[str]:
        ros_list = self.memory_list_srv().instances
        return [name.data for name in ros_list if "box" in name.data.lower()]

    def get_box_position(self, frame_id: str) -> TransformStamped:
        try:
            return self.buffer.lookup_transform(
                self.reference_frame, frame_id, rospy.Time(0)
            )
        except Exception as e:
            rospy.logerr(e)
            return None

    def find_box_in_laserscan(self):
        


if __name__ == "__main__":
    rospy.init_node("box_node")
    node = BoxNode()
    rospy.spin()
