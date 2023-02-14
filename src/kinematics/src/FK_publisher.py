#!/usr/bin/env python3
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, TransformListener, Buffer

class FKpublisher:
    def __init__(self) -> None:
        self.f = 10
        self.buffer = Buffer(1200.0)
        self.listener = TransformListener(self.buffer)
        self.br = TransformBroadcaster  
    