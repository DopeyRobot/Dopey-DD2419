#!/usr/bin/env python3
import rospy
from tf2_geometry_msgs import PoseStamped
from tf2_ros import TransformBroadcaster, Buffer, TransformListener, TransformStamped
from sensor_msgs.msg import LaserScan


class BoxNode:
    def __init__(self) -> None:
        self.laser_scan_sub = rospy.Subscriber(
            "/scan", LaserScan, self.laser_scan_callback
        )
        self.scan: LaserScan = rospy.wait_for_message("/scan", LaserScan, timeout=5.0)

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer)
        self.broadcaster = TransformBroadcaster()

    def laser_scan_callback(self, msg: LaserScan) -> None:
        self.scan = msg
        self.buffer.all_frames_as_yaml()  


if __name__ == "__main__":
    rospy.init_node("box_node")
    node = BoxNode()
    rospy.spin()
