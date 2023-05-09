#!/usr/bin/env python3
import math

import numpy as np
import py_trees as pt
import rospy
import tf2_geometry_msgs
import tf2_ros
from bounding_box_detection.srv import twoStrInPoseOut, twoStrInPoseOutRequest, closestObj, closestObjRequest
from geometry_msgs.msg import Point, PoseStamped
from occupancy_grid.srv import isOccupied, isOccupiedRequest, isOccupiedResponse

# class give_path(pt.behaviour.Behaviour):
from kinematics.srv import GripStrength, GripStrengthRequest, GripStrengthResponse
from nav_msgs.msg import OccupancyGrid, Path
from play_tunes.srv import playTune, playTuneRequest, playTuneResponse
from planning.srv import lastAngle, lastAngleRequest, lastAngleResponse
from robp_msgs.msg import DutyCycles
from std_msgs.msg import Bool, Empty, String
from std_srvs.srv import Trigger, TriggerRequest, TriggerResponse
import tf_conversions

class give_path(pt.behaviour.Behaviour):
    def __init__(self, exploring:bool= True):
        super().__init__("Give path!")
        self.exploring = exploring
        self.goal_threshold =0.2
        self.name = "give_path"
        self.goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=10)
        self.path_sub = rospy.Subscriber("/path_topic", Path, self.path_callback)
        self.ready_for_pose_sub = rospy.Subscriber(
            "/ready_for_pose", Bool, self.ready_for_path_callback
        )
        self.ready_for_pose_pub = rospy.Publisher(
            "/ready_for_pose", Bool, queue_size=1, latch=True
        )
        self.ready_for_new_path = rospy.Publisher(
            "/ready_for_new_path", Bool, queue_size=1, latch=True
        )
        self.reset_sub = rospy.Subscriber(
            "/reset_path_planning", Bool, self.reset_cb
        )

        self.pose_service = rospy.ServiceProxy("/get_object_pose", twoStrInPoseOut)
        self.playtune_service = rospy.ServiceProxy("/playTune", playTune)

        self.current_obj_id_sub = rospy.Subscriber("/current_obj_id", String, self.current_obj_id_cb)
        self.current_obj_id = String()

        self.path = None  # Path()
        self.ready_for_pose = Bool()
        self.ready_for_path = True
        self.pose_to_send = 0
        self.tooFar = True
        # become a behaviour

        # self.update()

    def current_obj_id_cb(self, msg):
        self.current_obj_id = msg

    def path_callback(self, msg):
        self.path = msg

    def ready_for_path_callback(self, msg):
        self.ready_for_pose = msg.data

    def reset_cb(self,msg):
        self.__init__(self.exploring)

    def check_euc_dist(self):
        dist = self.get_distance_to_goal()
        print("dist:",dist)
        if dist < 0.5:
            self.tooFar = False
            self.ready_for_pose = True
            self.ready_for_pose_pub.publish(self.ready_for_pose)

        else:
            self.tooFar = True

    def update(self):
        # print(f"is exploring {self.exploring}")
        # print(self.ready_for_pose)
        # print("ready for path in give_path:", self.ready_for_path)
        

        if self.path is not None:
            self.check_euc_dist()
            print("tooFar:",self.tooFar)
            print("pose_to_send:",self.pose_to_send)
            print("len of path:",len(self.path.poses))
            print("ready for pose:",self.ready_for_pose)
            # dist = self.get_distance_to_goal()

            # if not self.exploring and  dist < self.goal_threshold:
            #     self.__init__(self.exploring)  # path = None
            #     self.pose_to_send = 0
            #     self.ready_for_path = True
            #     self.ready_for_new_path.publish(self.ready_for_path)
            #     print("👉🥺👈, close enough")
            #     req = playTuneRequest(String("underwater"))
            #     self.playtune_service(req)
            #     return pt.common.Status.SUCCESS


            if self.ready_for_pose and self.pose_to_send < len(
                self.path.poses) and self.tooFar: 
                print("Condition 1")
                # and self.path.poses != []:
                # print('Ready for new pose! Sending RUNNING in tree')
                self.ready_for_pose = False
                self.ready_for_pose_pub.publish(self.ready_for_pose)
                self.goal_pub.publish(self.path.poses[self.pose_to_send])
                self.pose_to_send = self.pose_to_send + 1

                return pt.common.Status.RUNNING

            elif self.ready_for_pose and not self.tooFar:#nd self.pose_to_send == len(self.path.poses) :
                print("Condition 2")
                self.__init__(self.exploring)  # path = None
                self.pose_to_send = 0
                self.ready_for_path = True
                self.ready_for_new_path.publish(self.ready_for_path)
                print("Reached final pose, sending SUCCESS in tree")
                return pt.common.Status.SUCCESS

            else:
                print("Condition 3")
                self.ready_for_path = False
                self.ready_for_new_path.publish(self.ready_for_path)
                # print('Moving to next pose in path array! Sending RUNNING in tree')
                return pt.common.Status.RUNNING

        else:
            # print("Waiting for path, none given yet! Sending RUNNING in tree")
            self.ready_for_new_path.publish(self.ready_for_path)
            return pt.common.Status.RUNNING
        # else:
        #     print("sending final RUNNING")
        #     return pt.common.Status.RUNNING
    
    def get_distance_to_goal(self) -> float:
        ref_frame = String("base_link")
        req = twoStrInPoseOutRequest()
        req.str1 = ref_frame
        req.str2 = self.current_obj_id
        print(self.current_obj_id)
        # req = twoStrInPoseOutRequest(ref_frame, self.current_obj_id)
        object_pose = self.pose_service(req)
        x = object_pose.pose.pose.position.x
        y = object_pose.pose.pose.position.y
        dist = np.sqrt((x) ** 2
            + (y) ** 2)
        
        if not self.exploring:
            print(f"current dist to goal = {dist}")
        return dist


class FrontierExploration(pt.behaviour.Behaviour):
    def __init__(self):
        self.name = "exploration"
        rospy.Subscriber("/occupancygrid", OccupancyGrid, self.map_callback)
        self.publish_goal = rospy.Publisher(
            "/send_goal", PoseStamped, queue_size=1, latch=True
        )
        self.subcribe_ready_for_path = rospy.Subscriber(
            "/ready_for_new_path", Bool, self.ready_for_path_callback
        )
        # self.publish_frontier = rospy.Publisher('/frontier_point', PoseStamped, queue_size=1)

        self.buffer = tf2_ros.Buffer(rospy.Duration(100.0))
        rospy.sleep(0.5)
        self.listener = tf2_ros.TransformListener(self.buffer)
        rospy.sleep(0.5)

        self.occupancy_grid = None
        self.ready_for_path = True

        # become a behaviour
        super(FrontierExploration, self).__init__("Find frontier!")
        # self.update()

    def ready_for_path_callback(self, msg):
        self.ready_for_path = msg.data

    def map_callback(self, msg):
        self.map_data = msg
        self.occupancy_grid = np.asarray(self.map_data.data, dtype=np.int8).reshape(
            self.map_data.info.height, self.map_data.info.width
        )
        # print("Inside map callback")
        # print(self.occupancy_grid)

    def identify_froniters(self):

        frontier_cells = []
        if self.occupancy_grid is not None:
            for i in range(1, self.occupancy_grid.shape[0] - 1):
                for j in range(1, self.occupancy_grid.shape[1] - 1):
                    if (
                        self.occupancy_grid[i][j] == 0
                        and np.sum(self.occupancy_grid[i - 1 : i + 2, j - 1 : j + 2])
                        < 0
                    ):
                        x = (
                            (j - 0.5) * self.map_data.info.resolution
                            + self.map_data.info.origin.position.x
                        )
                        y = (
                            (i - 0.5) * self.map_data.info.resolution
                            + self.map_data.info.origin.position.y
                        )

                        frontier_cells.append((x, y))

        return frontier_cells

    def euclidean_distance(self, x1, y1, x2, y2):

        return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

    def get_current_position(self):
        base_link_origin = PoseStamped()
        base_link_origin.header.stamp = rospy.Time.now()

        transform_to_map = self.buffer.lookup_transform(
            "map", "base_link", base_link_origin.header.stamp, rospy.Duration(1)
        )
        baseInMapPose = tf2_geometry_msgs.do_transform_pose(
            base_link_origin, transform_to_map
        )

        return baseInMapPose

    def distance2frontier(self):
        current_position = self.get_current_position()
        current_x = current_position.pose.position.x
        current_y = current_position.pose.position.y
        frontier_cells = self.identify_froniters()
        # print(frontier_cells)

        distances = []
        for cell in frontier_cells:
            dist = self.euclidean_distance(current_x, current_y, cell[0], cell[1])
            distances.append(dist)

        # closest frontier cell
        if len(distances) != 0:
            min_distance_idx = np.argmin(distances)
            closest_frontier = frontier_cells[min_distance_idx]

            return closest_frontier
        else:
            return None

    def update(self):
        # while not rospy.is_shutdown():
        # FIX THE self.goal = None
        frontier_to_publish = PoseStamped()
        frontier_to_publish.header.frame_id = "map"
        frontier_to_publish.header.stamp = rospy.Time.now()
        closest_frontier = self.distance2frontier()
        if closest_frontier is not None:
            frontier_to_publish.pose.position.x = closest_frontier[0]
            frontier_to_publish.pose.position.y = closest_frontier[1]

            if self.ready_for_path:
                self.publish_goal.publish(frontier_to_publish)
                print("Sending current frontier goal, sending SUCCESS in tree")
                # print("ready for path in explore:", self.ready_for_path)
                # self.publish_frontier.publish(frontier_to_publish)
                return pt.common.Status.SUCCESS
            else:
                # Currently moving to a frontier, not ready to publish a new one
                # print("running expl 1")
                return pt.common.Status.RUNNING
        else:
            # Not yet found a frontier
            # print("running expl 2")

            return pt.common.Status.RUNNING


class MoveArmToPickupFront(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__("Moving arm to pickup front")
        self.move_srv = rospy.ServiceProxy("/pickup/front", Trigger)
        rospy.wait_for_service("/pickup/front", timeout=5)
        self.tried = False
        self.done = False
        self.resp = None

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        if not self.tried:
            self.tried = True
            self.resp: TriggerResponse = self.move_srv(TriggerRequest())
            return pt.common.Status.RUNNING
        if self.resp.success:
            self.done = True
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def reset(self):
        self.tried = False
        self.done = False
        self.resp = None


class MoveArmToHome(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__("Moving arm to home")
        self.move_srv = rospy.ServiceProxy("/home", Trigger)
        rospy.wait_for_service("/home", timeout=5)
        self.tried = False
        self.done = False
        self.resp = None

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        if not self.tried:
            self.tried = True
            self.resp: TriggerResponse = self.move_srv(TriggerRequest())
            return pt.common.Status.RUNNING
        if self.resp.success:
            self.done = True
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def reset(self):
        self.tried = False
        self.done = False
        self.resp = None


class MoveArmToUnfold(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__("Moving arm to home")
        self.move_srv = rospy.ServiceProxy("/unfold", Trigger)
        rospy.wait_for_service("/unfold", timeout=5)
        self.tried = False
        self.done = False
        self.resp = None

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        if not self.tried:
            self.tried = True
            self.resp: TriggerResponse = self.move_srv(TriggerRequest())
            return pt.common.Status.RUNNING
        if self.resp.success:
            self.done = True
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def reset(self):
        self.tried = False
        self.done = False
        self.resp = None


class MoveArmToDrop(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__("Moving arm to drop pos")
        self.move_srv = rospy.ServiceProxy("/drop", Trigger)
        rospy.wait_for_service("/drop", timeout=5)
        self.tried = False
        self.done = False
        self.resp = None

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        if not self.tried:
            self.tried = True
            self.resp: TriggerResponse = self.move_srv(TriggerRequest())
            return pt.common.Status.RUNNING
        if self.resp.success:
            self.done = True
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def reset(self):
        self.tried = False
        self.done = False
        self.resp = None


class MoveArmToTray(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__("Moving arm to tray")
        self.move_srv = rospy.ServiceProxy("/pickup/tray", Trigger)
        rospy.wait_for_service("/pickup/tray", timeout=5)
        self.tried = False
        self.done = False
        self.resp = None

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        if not self.tried:
            self.tried = True
            self.resp: TriggerResponse = self.move_srv(TriggerRequest())
            return pt.common.Status.RUNNING
        if self.resp.success:
            self.done = True
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def reset(self):
        self.tried = False
        self.done = False
        self.resp = None


class OpenGripper(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__("Opening gripper")
        self.move_srv = rospy.ServiceProxy("/gripper/open", Trigger)
        rospy.wait_for_service("/gripper/open", timeout=5)
        self.tried = False
        self.done = False
        self.resp = None

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        if not self.tried:
            self.tried = True
            self.resp: TriggerResponse = self.move_srv(TriggerRequest())
            return pt.common.Status.RUNNING
        if self.resp.success:
            self.done = True
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def reset(self):
        self.tried = False
        self.done = False
        self.resp = None


class CloseGripper(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__("Closing gripper")
        self.move_srv = rospy.ServiceProxy("/gripper/close", GripStrength)
        rospy.wait_for_service("/gripper/close", timeout=5)
        self.tried = False
        self.done = False
        self.resp = None

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        if not self.tried:
            self.tried = True
            req = GripStrengthRequest(String("cube"))
            self.resp: TriggerResponse = self.move_srv(req)
            return pt.common.Status.RUNNING
        if self.resp.success:
            self.done = True
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def reset(self):
        self.tried = False
        self.done = False
        self.resp = None


class PickupToTarget(pt.behaviour.Behaviour):
    """
    pickup to target that is a poststamped publish to the pickup_goal topic in base_link frame
    """

    def __init__(self):
        super().__init__("Pickup to goal")
        self.move_srv = rospy.ServiceProxy("/full_pickup_pose", Trigger)
        rospy.wait_for_service("/full_pickup_pose", timeout=5)
        self.tried = False
        self.done = False
        self.resp = None

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        if not self.tried:
            self.tried = True
            self.resp: TriggerResponse = self.move_srv(TriggerRequest())
            return pt.common.Status.RUNNING
        if self.resp.success:
            self.done = True
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE

    def reset(self):
        self.tried = False
        self.done = False
        self.resp = None


class Wait(pt.behaviour.Behaviour):
    def __init__(self, duration: int):
        super().__init__("wait")
        self.duration = duration
        self.done = False

    def update(self):
        if self.done:
            return pt.common.Status.SUCCESS
        rospy.sleep(self.duration)
        self.done = True
        return pt.common.Status.SUCCESS

    def reset(self):
        self.done = False


class Reset(pt.behaviour.Behaviour):
    def __init__(
        self,
        behaviors: list,
    ):
        super().__init__("reset")
        self.behaviors = behaviors

    def update(self):
        for behavior in self.behaviors:
            behavior.reset()
        return pt.common.Status.SUCCESS


class playTuneBehaviour(pt.behaviour.Behaviour):
    def __init__(self, tune_name: str):
        super().__init__("Play tune : " + tune_name)
        rospy.loginfo("Initialising playing sound behaviour for " + tune_name)
        self.tune_name = tune_name
        self.playTune_client = rospy.ServiceProxy("playTune", playTune)
        rospy.wait_for_service("playTune", timeout=2)

    def update(self):
        self.playTune_client(String(self.tune_name))
        return pt.common.Status.SUCCESS
    



class ReturnKnownMapPercent(pt.behaviour.Behaviour):
    def __init__(self, p):
        self.name = "ReturnKnownMapPercent"
        rospy.Subscriber("/occupancygrid", OccupancyGrid, self.map_callback)
        # self.playTune_client = rospy.ServiceProxy("playTune", playTune)
        # rospy.wait_for_service("playTune", timeout=2)
        self.ready_for_new_path_pub = rospy.Publisher(
            "/ready_for_new_path", Bool, queue_size=1, latch=True
        )
        self.reset_pub = rospy.Publisher(
            "/reset_path_planning", Bool
        )

        self.occupancy_grid = None
        self.p = p
        self.number_of_total_map_cells = None
        self.collectedFirstMap = False

        super(ReturnKnownMapPercent, self).__init__(
            "Explored space > " + str(self.p * 100) + "%"
        )

    def map_callback(self, msg):
        self.map_data = msg
        self.occupancy_grid = np.asarray(self.map_data.data, dtype=np.int8).reshape(
            self.map_data.info.height, self.map_data.info.width
        )
        if not self.collectedFirstMap:
            self.number_of_total_map_cells = np.count_nonzero(
                self.occupancy_grid == -1
            )  # number of cells to explore initially, will only run once
            self.collectedFirstMap = True

    def update(self):
        condition = self.occupancy_grid == -1
        number_of_unexplored_elements = np.count_nonzero(condition)

        # number_of_elements = self.occupancy_grid.shape[0]*self.occupancy_grid.shape[1]
        percentage_of_unexplored = (
            number_of_unexplored_elements / self.number_of_total_map_cells
        )
        percentage_of_unexplored = 1-percentage_of_unexplored
        print(percentage_of_unexplored)
        print(self.p)
        if percentage_of_unexplored >= self.p:
            # self.playTune_client(String("gothim"))
            # NOTE: LOOK HERE; test to clear the path lpannign before moving onto main mission
            true = Bool()
            true.data = True
            self.reset_pub.publish(true)
            self.ready_for_new_path_pub.publish(true)
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.FAILURE


class StopRobot(pt.behaviour.Behaviour):
    def __init__(self, duration):
        self.name = "StopRobot"
        self.duty_pub = rospy.Publisher("/motor/duty_cycles", DutyCycles, queue_size=10)
        self.duration = duration

        super(StopRobot, self).__init__("Stop Robot for " + str(int(duration)) + " sec")

        self.start = rospy.Time.now()

    def update(self):
        if rospy.Time.now() - self.start > rospy.Duration(self.duration):
            return pt.common.Status.SUCCESS
        else:
            msg = DutyCycles()
            msg.duty_cycle_left = 0
            msg.duty_cycle_right = 0
            self.duty_pub.publish(msg)
            return pt.common.Status.RUNNING

    def initialise(self):
        print("Resetting start time")
        self.start = rospy.Time.now()


class GetClosestObjectPose(pt.behaviour.Behaviour):
    def __init__(self):
        self.name = "desired pose"
        self.publish_goal = rospy.Publisher(
            "/send_goal", PoseStamped, queue_size=1, latch=True
        )
        self.subcribe_ready_for_path = rospy.Subscriber(
            "/ready_for_new_path", Bool, self.ready_for_path_callback
        )
        self.publish_obj_id = rospy.Publisher(
            "/current_obj_id", String, queue_size=1, latch=True
        )

        self.publisher_focus_frame_id = rospy.Publisher(
            "/current_focus_id", String, queue_size=1, latch= True
        )
        self.occupancy_client = rospy.ServiceProxy("/occupied_service", isOccupied)

        self.ready_for_path = True

        self.getPose_client = rospy.ServiceProxy("/get_closest_obj", closestObj)
        rospy.wait_for_service("/get_closest_obj", timeout=2)

        # become a behaviour
        super(GetClosestObjectPose, self).__init__("Get pick up pose")

    def ready_for_path_callback(self, msg):
        self.ready_for_path = msg.data

    def update(self):
        req = closestObjRequest()
        req.ref_frame = String("map")  # frame_id
        req.desired_class = String("no_box")  # object class ball/plushie/box
        desPose = self.getPose_client(req)  # assume awlays a pose is given
        if desPose.foundId == "poop": # no object found
            print("Poop No object found D:<")
            return pt.common.Status.FAILURE

        elif self.ready_for_path:
            # desPose.pose.pose.position.x += 0.1 #NOTE: look here, offset to not crash into object
            gen = self.spiral_points()
            orig_desPose = desPose
            ocReq = isOccupiedRequest(desPose.pose)
            poseIsOccupied = self.occupancy_client(ocReq)
            x,y = next(gen)
            while poseIsOccupied.isOccBool and np.sqrt(x**2+y**2) < 0.2:
                
                desPose.pose.pose.position.x = orig_desPose.pose.pose.position.x + x
                desPose.pose.pose.position.y = orig_desPose.pose.pose.position.y + y
                ocReq = isOccupiedRequest(desPose.pose)
                poseIsOccupied = self.occupancy_client(ocReq)
                x,y = next(gen)


            self.publish_goal.publish(desPose.pose)
            print("Sending current desired pose\n")
            self.publish_obj_id.publish(desPose.foundId)
            self.publisher_focus_frame_id.publish(desPose.foundId)
            print("Sending current object id: " + desPose.foundId.data, "\n:) Sending SUCCESS in tree")
            return pt.common.Status.SUCCESS

        else:
            return pt.common.Status.RUNNING
            
    def spiral_points(self, arc=0.1, separation=0.1):
        """generate points on an Archimedes' spiral
        with `arc` giving the length of arc between two points
        and `separation` giving the distance between consecutive 
        turnings
        - approximate arc length with circle arc at given distance
        - use a spiral equation r = b * phi
        """
        def p2c(r, phi):
            """polar to cartesian
            """
            return (r * np.cos(phi), r * np.sin(phi))

        # yield a point at origin
        yield (0, 0)

        # initialize the next point in the required distance
        r = arc
        b = separation / (2 * np.pi)
        # find the first phi to satisfy distance of `arc` to the second point
        while True:
            phi = float(r) / b
            yield p2c(r, phi)
            # advance the variables
            # calculate phi that will give desired arc length at current radius
            # (approximating with circle)
            phi += float(arc) / r
            r = b * phi


class GetBoxPose(pt.behaviour.Behaviour):
    def __init__(self):
        self.name = "desired pose"
        self.publish_goal = rospy.Publisher(
            "/send_goal", PoseStamped, queue_size=1, latch=True
        )
        self.subcribe_ready_for_path = rospy.Subscriber(
            "/ready_for_new_path", Bool, self.ready_for_path_callback
        )

        self.cur_obj_subscriber = rospy.Subscriber(
            "/current_obj_id", String, self.current_object_callback
        )

        self.publisher_focus_frame_id = rospy.Publisher(
            "/current_focus_id", String, queue_size=1, latch= True
        )

        self.current_object = None
        self.ready_for_path = True

        self.box_dict = {"ball": 2, "cube": 2, "plushie": 2}

        self.getPose_client = rospy.ServiceProxy("/get_object_pose", twoStrInPoseOut)
        rospy.wait_for_service("/get_object_pose", timeout=2)

        # become a behaviour
        super(GetBoxPose, self).__init__("Get drop off pose")

    def ready_for_path_callback(self, msg):
        self.ready_for_path = msg.data

    def current_object_callback(self, msg):
        object_type = msg.data.split("_")[0]
        if object_type != "cube" and object_type != "ball":
            object_type = "plushie"
        self.current_object = object_type

    def update(self):
        if self.current_object is not None:
            landmark_id = self.box_dict[self.current_object]
            target_frame = "Landmark" + str(landmark_id)
            req = twoStrInPoseOutRequest()
            req.str1.data = "map"  # frame_id
            req.str2.data = target_frame  # object class ball/plushie/box
            desPose = self.getPose_client(req).pose  # assume awlays a pose is given

            if self.ready_for_path:
                self.publish_goal.publish(desPose)
                self.publisher_focus_frame_id.publish(target_frame)
                print("Sending current desired pose, sending SUCCESS in tree")
                return pt.common.Status.SUCCESS

            else:
                return pt.common.Status.RUNNING
        else:
            return pt.common.Status.RUNNING

class LookatCurrentFocus(pt.behaviour.Behaviour):
    def __init__(self):
        rospy.Subscriber("/current_focus_id", String, self.focus_cb)
        super().__init__("Look at current focus")
        self.targetframe = None
        self.ready_for_pose_sub = rospy.Subscriber(
            "/ready_for_pose", Bool, self.ready_for_pose_callback
        )
        self.move2goalDone = Bool()
        self.move2goalDone.data = False
        self.getpose_client= rospy.ServiceProxy("/get_object_pose", twoStrInPoseOut)
        rospy.wait_for_service("playTune", timeout=2)
        self.pose_sent = False

        self.goal_pub = rospy.Publisher("/goal", PoseStamped, queue_size=10)

    def focus_cb(self, msg):
        self.targetframe = msg.data

    def ready_for_pose_callback(self, msg):
        self.move2goalDone = msg.data

    def update(self):
        if not self.pose_sent:
            # headingpose = PoseStamped()
            # headingpose.header.stamp = rospy.Time.now()
            # headingpose.header.frame_id = "map"
        #         poly_req = PolyCheckRequest()
        #         poly_req.point_at_infinity = self.pinf
        #         poly_req.point_of_interest = [self.get_x_pos(x), self.get_y_pos(y)]
        #         poly_resp = self.polygon_client.call(poly_req)
            targetinbaselink_req = twoStrInPoseOutRequest()
            targetinbaselink_req.str1 = String("base_link")
            targetinbaselink_req.str2 = String(self.targetframe)
            targetinbaselink_pose = self.getpose_client(targetinbaselink_req)
            baselinkinmap_req = twoStrInPoseOutRequest()
            baselinkinmap_req.str1 = String("map")
            baselinkinmap_req.str2 = String("base_link")
            baselinkinmap_pose = self.getpose_client(baselinkinmap_req)

            tx = targetinbaselink_pose.pose.pose.position.x
            ty = targetinbaselink_pose.pose.pose.position.y

            angle_correction = math.pi/2 - np.arctan2(ty, tx)

            currentheading_euler = tf_conversions.transformations.euler_from_quaternion([baselinkinmap_pose.pose.pose.orientation.x, baselinkinmap_pose.pose.pose.orientation.y, baselinkinmap_pose.pose.pose.orientation.z, baselinkinmap_pose.pose.pose.orientation.w])[2]
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, angle_correction + currentheading_euler)

            baselinkinmap_pose.pose.pose.orientation.x = q[0]
            baselinkinmap_pose.pose.pose.orientation.y = q[1]
            baselinkinmap_pose.pose.pose.orientation.z = q[2]
            baselinkinmap_pose.pose.pose.orientation.w = q[3]
            self.pose_sent = True
            self.goal_pub.publish(baselinkinmap_pose.pose)
            return pt.common.Status.RUNNING
        
        elif self.move2goalDone:
            return pt.common.Status.SUCCESS
        else:
            return pt.common.Status.RUNNING
        
class SendGoalToArm(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__("SendGoal")
        self.getpose_client= rospy.ServiceProxy("/get_object_pose", twoStrInPoseOut)
        self.cur_obj_subscriber = rospy.Subscriber(
            "/current_obj_id", String, self.current_object_callback
        )
        self.pickup_goal_publisher = rospy.Publisher(
            "/pickup_goal", PoseStamped, queue_size=10
        )
        self.current_obj = None
        self.ref_frame = String("base_link")


    def current_object_callback(self, msg):
        self.current_obj = msg

    def update(self):
        if self.current_obj is not None:
            req = twoStrInPoseOutRequest()
            req.str1 = self.ref_frame
            req.str2 = self.current_obj

            pose = self.getpose_client(req)

            self.pickup_goal_publisher.publish(pose.pose)

            return pt.common.Status.SUCCESS
        
        return pt.common.Status.RUNNING

class approach_goal(pt.behaviour.Behaviour):
    def __init__(self):
        super().__init__("Appraoching goal!")

        self.lastAngle_client = rospy.ServiceProxy("/lastAngle", lastAngle)


        self.cur_obj_subscriber = rospy.Subscriber(
            "/current_obj_id", String, self.current_object_callback
        )
        self.current_obj = None
        self.ref_frame = String("base_link")
    def reset(self):
        return None
    def current_object_callback(self, msg):
        self.current_obj = msg
    
    def update(self):
        rospy.loginfo("Approaching Goal")
        if self.current_obj is not None:
            req = lastAngleRequest()

            # req.str1 = self.ref_frame #targetframe, dont need: defined as baselink in movetogoal
            req.goal_frameid = self.current_obj #urrent object frame
            self.lastAngle_client(req)
            print("last angle client success")
            return pt.common.Status.SUCCESS
        return pt.common.Status.RUNNING
    

