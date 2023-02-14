import rospy
from sensor_msgs.msg import JointState
from kinematics_solver import KinematicsSolver


class ArmSimulator:
    """
    class to simulate the arm in RViz
    """

    def __init__(self):
        self.joint_state_publisher = rospy.Publisher(
            "/joint_states", JointState, queue_size=10
        )
        self.kine = KinematicsSolver()
