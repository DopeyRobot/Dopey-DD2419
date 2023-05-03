#!/usr/bin/env python3
from collections import Counter
from enum import Enum
import numpy as np
from dataclasses import dataclass
import rospy
from tf2_ros import Buffer, TransformListener, TransformBroadcaster, TransformStamped
from std_msgs.msg import String
from std_srvs.srv import Empty, EmptyRequest, EmptyResponse
from bounding_box_detection.srv import (
    add2ShortTerm,
    add2ShortTermRequest,
    add2ShortTermResponse,
    instanceNames,
    instanceNamesRequest,
    instanceNamesResponse,
    setLocation,
    setLocationRequest,
    setLocationResponse,
    twoStrInPoseOut,
    twoStrInPoseOutRequest,
    twoStrInPoseOutResponse,
    closestObj,
    closestObjRequest,
    closestObjResponse,
)
from workspace.srv import PolyCheck, PolyCheckRequest
from play_tunes.srv import playTune, playTuneRequest
from bounding_box_detection.msg import StringArray
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs import PoseStamped as Tf2PoseStamped


class Locations(Enum):
    MAP = "map"
    BOX = "box"
    GRIPPER = "gripper"
    TRAY = "tray"

class Plushies(Enum):
    K = "kiki"
    M = "muddles"
    B = "binky"
    S = "slush"
    H = "hugo"
    O = "oakie"


@dataclass
class LongTermInstance:
    instance_name: str
    position: np.ndarray
    last_time_seen: rospy.Time
    location:Locations

    def __str__(self) -> str:
        return f"name: {self.instance_name} \nposition: {self.position}, \nlast_time_seen: {self.last_time_seen} \n\n"


# @dataclass
# class DetectionObject:
#     class_name:str
#     position:PoseStamped #contains position in map frame and time of detection


class ShortTermMemory:
    """The detection buffer is a filter to reject random false detections.
    It keeps track of how many times an instance has been detected and how many times a class has been detected."""

    def __init__(
        self, same_obj_threshold=0.05, time_threshold=rospy.Duration(5)
    ) -> None:
        self.instances_detected_counter = (
            Counter()
        )  # keeps track of how many times the same instance has been detected
        self.class_counter = (
            Counter()
        )  # keeps track of how many times a new element of every class has been detected
        self.last_time_seen = {}
        self.average_position = {}
        self.same_obj_threshold = same_obj_threshold
        self.time_threshold = time_threshold

    def get_next_instance_name(self, class_name):
        return class_name + "_" + str(self.class_counter[class_name])

    def get_instance_position(self, instance_name):
        return self.average_position[instance_name]

    def get_class_name(self, instance_name: str):
        return instance_name.split("_")[0]

    def check_position(self, position):
        keys = []
        for key, value in self.average_position.items():
            if np.linalg.norm(position - value) < self.same_obj_threshold:
                keys.append(key)
        keys = sorted(
            keys, key=lambda x: np.linalg.norm(position - self.average_position[x])
        )
        return keys

    def check_time(self, timestamp):
        """Deletes all instances that have not been seen for more than time_threshold"""
        keys = []
        for key, value in self.last_time_seen.items():
            if timestamp - value > self.time_threshold:
                keys.append(key)

        for key in keys:
            del self.instances_detected_counter[key]
            del self.last_time_seen[key]
            del self.average_position[key]
            # do we want to decrease the class counter as well?
            # Problem: maybe we just deleted the last instance of a class but we want to add to a higher number.
            # What if everytime we called this we passed it the external counter?

    def _add(self, class_name, instance_name, position, timestamp):
        """Increments the counter for the instance (whther it exists or not) and updates the average position"""

        if instance_name in self.instances_detected_counter:
            self.average_position[instance_name] = 0.5*position +0.5*self.average_position[instance_name]
            #(self.instances_detected_counter[instance_name]/ (self.instances_detected_counter[instance_name] + 1)) * 
            #(self.average_position[instance_name]+ position / (self.instances_detected_counter[instance_name]))
            self.instances_detected_counter[
                instance_name
            ] += 1  # we saw this instance again
            self.last_time_seen[instance_name] = timestamp

        else:
            self.instances_detected_counter[instance_name] = 1
            self.average_position[instance_name] = position
            self.class_counter[
                class_name
            ] += 1  # we saw a new instance belonging to this class
            self.last_time_seen[instance_name] = timestamp

    def add(self, class_name, position, timestamp):
        self.check_time(
            timestamp
        )  # delete old instances (possible fix: only delete them if is was only seen for e.g. 3 frames)
        keys = self.check_position(position)

        for key in keys:
            if len(keys) != 0 and self.get_class_name(key) == class_name:
                # print(f"adding {class_name} at {position} to {key}")
                self._add(class_name, key, position, timestamp)
                return

        # print(f"adding {class_name} at {position} to new instance")
        instance_name = self.get_next_instance_name(class_name)
        self._add(class_name, instance_name, position, timestamp)


# The LongTermMememory database will add an object to its list IF AND ONLY IF the detection buffer has identified an intance more than N times,
# This LongTermMemory database will check if it exists in it already by comparing class name and position (not instance name)-->
# -->There will be different numeration inside the different buffers:
#  so for example kiki_1 in the long term memory can be kiki 14 in the short detection buffer!

# If this istance does exist in the LongTermMemory, it will update it's position and timestamp
# If it has been detected more than N times in the DetectionBuffer but it doens't exist in the LongTermMemory, it will add it to the LongTermMemory

# The longTermMemory database will keep track of the detected objects' names, where they are
# (M = Map, T=Tray, G=Grip, B = box (or maybe they can be forgoten when they are in a box?)), and their timestamps
# The longTermMemory database will only delete objects from its list when they are outdated:
# e.g., position does not correspond with TF tree, the object was picked up, etc.


class LongTermMemory:
    """Stores the objects that have been detected more than N times in the DetectionBuffer"""

    def __init__(self, frames_needed_for_reconition=20, same_obj_threshold=0.2, other_obj_threshold=0.15) -> None:
        self.class_counter = (Counter())  # keeps track of how many times a new element of every class has been detected
        self.instances_in_memory = []
        self.locations = {}  # is the instance in the Map, Tray, Grip, Box?
        self.last_time_seen = {}
        self.positions = {}
        self.same_obj_threshold = same_obj_threshold
        self.other_obj_threshold = other_obj_threshold
        self.min_frames_needed = frames_needed_for_reconition  # how many times the object has to be detected in the DetectionBuffer to be added to the LongTermMemory

    def get_next_instance_name(self, class_name):
        return class_name + "_" + str(self.class_counter[class_name])

    def get_class_name(self, instance_name: str):
        return instance_name.split("_")[0]

    def check_position(self, position):
        """Checks if there is an instance in the long term memory that is close enough
        to the position we want to add and returns the name of the closest one"""
        keys = []
        for key, value in self.positions.items():
            if np.linalg.norm(position - value) < self.same_obj_threshold:
                keys.append(key)
        keys = sorted(keys, key=lambda x: np.linalg.norm(position - self.positions[x]))

        if len(keys) != 0:
            return keys[0]
        else:
            return None
            # do we want to decrease the class counter as well? Problem: maybe we just deleted the last instance of a class but we want to add to a higher number.

    def _updateMemory(self, timestamp, db, class_name, instance_name, position):
        """Updates an old or creates a new element for the Long term memory"""
        # If other instances of the same class exist in the long term memory check if they are close enough to the one we want to add.
        # If they are, update their position and timestamp:
        if class_name in [
            self.get_class_name(instance) for instance in self.instances_in_memory
        ]:
            # TODO set different locations (M,T,G,B) depending on where the obkect is
            closest_instance_in_lt_memory = self.check_position(
                db.average_position[instance_name]
            )
            if closest_instance_in_lt_memory is not None:
                self.locations[
                    closest_instance_in_lt_memory
                ] = "map"  # !!!FOR NOW ONLY OBJECTS IN MAP ARE STORED IN THE LONG TERM MEMORY!!!
                self.positions[closest_instance_in_lt_memory] = (position + self.positions[closest_instance_in_lt_memory])/2
                self.last_time_seen[closest_instance_in_lt_memory] = timestamp
                return None

        # if there is no instance in the long term memory of this class that is close enough to the one we want to add,
        # create a new instance and add it to the long term memory
        instance_name = self.get_next_instance_name(
            class_name
        )  # create a new instance name
        self.instances_in_memory.append(instance_name)
        self.positions[instance_name] = position
        self.locations[instance_name] = "map"
        self.last_time_seen[instance_name] = timestamp
        self.class_counter[
            class_name
        ] += 1  # we saw a new instance belonging to this class
        return instance_name

    def checkForObjectsToRemember(self, timestamp, db: ShortTermMemory):
        """Checks if there are objects in the ShortTermMemory that have been detected more than N times and adds them to the LongTermMemory"""
        new_names = []
        for db_instance, counter in db.instances_detected_counter.items():
            if counter > self.min_frames_needed:
                if self.too_crowded(db_instance, db):
                    continue
                new_name = self._updateMemory(
                    timestamp,
                    db,
                    self.get_class_name(db_instance),
                    db_instance,
                    db.get_instance_position(db_instance),
                )
                if new_name is not None:
                    new_names.append(new_name)
        return new_names

    def too_crowded(self, db_instance, db:ShortTermMemory):
        for lt_instance in self.instances_in_memory:
            if (
                np.linalg.norm(
                    db.get_instance_position(db_instance)
                    - self.positions[lt_instance]
                )
                < self.other_obj_threshold
            ):
                return True
        #!!!!NCOMMENT THE FOLLOWING LINES ONCE THE JAMMIN BRANCH AND THIS ONE ARE MERGED!!!!
        # # play sounds every time we either add or update an instance in the long term memory
        # try:
        #    play_tune = rospy.ServiceProxy('playTune', AudioService)
        #    play_tune(self.get_class_name(db_instance)
        # # TODO: "+ color" # e.g. for plushies: oakie, kiki, binky, ... but for balls & cubes: blueball, greenball, ..., greencube, woodencube, ...#
        #     )
        # except rospy.ServiceException as e:
        #     print("Service call failed: %s"%e)

    def __len__(self):
        return len(self.instances_in_memory)

    def __getitem__(self, key):
        name = self.instances_in_memory[key]
        return LongTermInstance(
            name,
            self.positions[name],
            self.last_time_seen[name],
            self.locations[name],
        )
    
    def get_instance(self, instance_name:str):
        return LongTermInstance(
            instance_name,
            self.positions[instance_name],
            self.last_time_seen[instance_name],
            self.locations[instance_name],
        )
    


class MemoryNode:
    def __init__(
        self,
        frames_needed_for_reconition=5,
        same_obj_threshold=0.2,
        time_threshold=rospy.Duration(10),
    ) -> None:
        self.f = 5
        self.db = ShortTermMemory(
            same_obj_threshold=same_obj_threshold, time_threshold=time_threshold
        )
        self.lt = LongTermMemory(frames_needed_for_reconition, same_obj_threshold)
        self.buffer = Buffer(rospy.Duration(1200.0))
        rospy.sleep(5)
        self.tranform_listener = TransformListener(self.buffer)
        self.tranform_broadcaster = TransformBroadcaster()
        self.new_names_pub = rospy.Publisher("/new_names", StringArray, queue_size=10)
        self.add2short_term_srv = rospy.Service(
            "/add2shortterm", add2ShortTerm, self.add2short_term_srv_cb
        )
        self.instances_in_LTM_srv = rospy.Service(
            "/instances_in_LTM", instanceNames, self.instances_in_LTM_srv_cb
        )
        self.set_location_srv = rospy.Service(
            "/change_location", setLocation, self.change_location_srv_cb
        )
        self.get_object_pose_srv = rospy.Service(
            "/get_object_pose", twoStrInPoseOut, self.get_object_pose_cb
        )
        self.get_closest_obj_srv = rospy.Service(
            "/get_closest_obj", closestObj, self.get_closest_obj_cb
        )
        self.play_tune_srv = rospy.ServiceProxy("playTune", playTune)

        # self.heading_srv = rospy.Service(
        #     "/heading_fix", FixHeading, self.heading_fix_cb 
        # )
        
        self.polygon_checker = rospy.ServiceProxy("/polygon_service", PolyCheck)
        self.camera_frame = "camera_color_optical_frame"

        self.run()

    def run(self):
        rate = rospy.Rate(self.f)
        while not rospy.is_shutdown():
            new_names = self.update_long_term(rospy.Time.now())
            self.publish_new_names(new_names)
            self.publish_long_term_memory()
            rate.sleep()

    def publish_new_names(self, new_names):
        if len(new_names) > 0:
            new_names_msg = StringArray()
            msg_list = []
            for name in new_names:
                name_msg = String()
                name_msg.data = name
                msg_list.append(name_msg)
            new_names_msg.array = msg_list
            self.new_names_pub.publish(new_names_msg)
        else:
            self.new_names_pub.publish(StringArray())


    def instances_in_LTM_srv_cb(self, req: instanceNamesRequest):
        resp_list = []
        for name in self.lt.instances_in_memory:
            resp_list.append(String(name))
        resp = instanceNamesResponse(resp_list)
        #print(resp)
        return resp

    def add2short_term_srv_cb(self, req: add2ShortTermRequest):
        class_name = req.class_name.data
        position = np.array(req.position)
        timestamp = req.stamp

        self.add_to_short_term(class_name, position, timestamp)
        return add2ShortTermResponse(True)
    
    def change_location_srv_cb(self, req: setLocationRequest):
        instance_name = req.frame_id.data
        loc = req.location.data # "map", "box", "gripper", "tray"
        if(loc in [e.value for e in Locations]):
            self.lt.locations[instance_name] = loc
            rospy.loginfo("Location of " + instance_name + " changed to " + loc.name)
        else:
            rospy.loginfo("Location name not recognized, try with map, box, tray, or gripper")
        return setLocationResponse(EmptyResponse())

    def add_to_short_term(
        self, class_name: str, position: np.array, timestamp: rospy.Time
    ):
        self.db.add(class_name, position, timestamp)

    def update_long_term(self, timestamp):
        new_names = self.lt.checkForObjectsToRemember(timestamp, self.db)
        return new_names

    def publish_to_tf(self, frame_name: str, position: np.ndarray):
        """
        Publish a transform from the camera frame to the map frame
        """
        t = TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"

        t.transform.translation.x = position[0]
        t.transform.translation.y = position[1]
        t.transform.translation.z = position[2]

        t.transform.rotation.x = 0.5
        t.transform.rotation.y = 0.5
        t.transform.rotation.z = 0.5
        t.transform.rotation.w = 0.5

        t.child_frame_id = frame_name
        try:
            self.tranform_broadcaster.sendTransform(t)
        except:
            print("sending transform to tf failed")

    def publish_long_term_memory(self):
        for instance_name in self.lt.instances_in_memory:
            instance = self.lt.get_instance(instance_name)
            self.publish_to_tf(
                instance_name,
                instance.position
                )
            
    # returns the pose of an object in the desired frame
    def get_object_pose_cb(self,req: twoStrInPoseOutRequest) -> PoseStamped:
        ref_frame_id = req.str1.data
        object_frame_id= req.str2.data
        return twoStrInPoseOutResponse(self.get_object_pose(ref_frame_id,object_frame_id))
    
    def get_object_pose(self, ref_frame_id:str, object_frame_id:str) -> PoseStamped:
        pose = PoseStamped()
        try:
            transform = self.buffer.lookup_transform(ref_frame_id, object_frame_id, rospy.Time(0) )
            pose.header.frame_id = transform.header.frame_id
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation.x = transform.transform.rotation.x
            pose.pose.orientation.y = transform.transform.rotation.y
            pose.pose.orientation.z = transform.transform.rotation.z
            pose.pose.orientation.w = transform.transform.rotation.w
            pose.header.stamp = transform.header.stamp
            # rospy.loginfo("pose of"+ frame_id + str(pose))
            return pose
        except Exception as e:
            rospy.loginfo("could not find transform for" + object_frame_id)
            rospy.logerr(e)
            return None
        
    def check_for_obj_type(self, instance_name: str, desired_class_of_obj:str) -> bool:
        if (desired_class_of_obj.lower() == self.lt.get_class_name(instance_name).lower() 
            or desired_class_of_obj.lower() == "all"
            or (desired_class_of_obj.lower() == "plushie" 
                and self.lt.get_class_name(instance_name).lower() in [e.value for e in Plushies])
            or (desired_class_of_obj.lower() == "no_box" and self.lt.get_class_name(instance_name).lower() !="box")):
            return True
        else:
            return False
    def obj_inside_map(self, object_frame_id:str)-> bool:
        poly_req = PolyCheckRequest()
        pose = self.get_object_pose("map", object_frame_id)
        poly_req.point_of_interest = [pose.pose.position.x, pose.pose.position.y]
        poly_resp = self.polygon_checker(poly_req)
        rospy.loginfo(f"object inside {poly_resp.poly_bool}")
        return poly_resp.poly_bool
    # find the closest object in the memory node of the given class and returns its pose in the ref_frame
    def get_closest_obj_cb(self, req: closestObjRequest) -> PoseStamped:
        """ Finds the closest instance in the long term memory
        if str2 == plushie--> returns closest plushie
                == kiki --> closest kiki (or any plushy name)
                == ball --> closest ball
                == cube --> closest cube 
                == box --> closest box
                == all --> closest object
                == no_box--> closest object that ain't a box"""
        ref_frame_id = req.ref_frame.data # first arg is the reference frame id
        desired_class_of_obj = req.desired_class.data # second arg is the class of the object ball/plushie/box
        closest_instance_pose = None
        poseOfRobot = self.get_object_pose(ref_frame_id, "base_link")
        name_of_closest_obj_found = "Nothing Found"
        dist = float("inf")
        prev_dist = float("inf")
        for instance_name in self.lt.instances_in_memory:
            if self.check_for_obj_type(instance_name, desired_class_of_obj) and self.lt.get_instance(instance_name).location.lower() == "map":
                poseOfObj = self.get_object_pose(ref_frame_id, instance_name)
                dist = self.get_distance(poseOfRobot,poseOfObj)
                if dist < prev_dist and self.obj_inside_map(instance_name):
                    prev_dist = dist
                    name_of_closest_obj_found = instance_name
                    closest_instance_pose = poseOfObj
        if closest_instance_pose is not None:
            rospy.loginfo(f"closest obstacle to base_link found is \"{name_of_closest_obj_found}\" at dist: {np.sqrt(prev_dist)}")
            self.play_tune_srv("!")
            return closestObjResponse(closest_instance_pose, name_of_closest_obj_found)
        else:
            rospy.loginfo("no obstacle found on the map")
            self.play_tune_srv("no_object_found") 
            failed_pose = PoseStamped()
            return closestObjResponse(failed_pose, "poop") # "poop" is the name of the object if there was no object found
        
    # def heading_fix_cb(self, req: FixHeadingRequest) -> float:
    #     ref_frame_id = req.ref_frame
    #     object_frame_id = req.object_frame
    #     pose_robot = self.get_object_pose("map", ref_frame_id)
    #     pose_object = self.get_object_pose("map", object_frame_id)

    #     #create while loop here and check what angle_error are
    #     angle_error = np.arctan2((pose_object.y - pose_robot.y), (pose_robot.x - pose_robot.x))
        
    #     return FixHeadingResponse(angle_error)
        
    def get_distance(self, pose1, pose2) -> float:
        """Returns the square of the distance between two poses"""
        return((pose1.pose.position.x - pose2.pose.position.x) ** 2
            + (pose1.pose.position.y - pose2.pose.position.y) ** 2
            + (pose1.pose.position.z - pose2.pose.position.z) ** 2)

if __name__ == "__main__":
    # import time

    # db = ShortTermMemory()
    # lt = LongTermMemory(1, 0.05)
    # start = time.time()
    # print(db.get_class_name("person_0"))
    # print(db.get_next_instance_name("person"))
    # db.add("person", np.array([0, 0, 0]), 4)
    # db.add("person", np.array([0.1, 0.1, 0.1]), 4)
    # print(db.instances_detected_counter)
    # db.add("person", np.array([0.1, 0.1, 0.1]), 4)
    # print(db.instances_detected_counter)
    # db.add("person", -np.array([0.1, 0.1, 0.1]), 4)
    # print(db.instances_detected_counter)
    # db.add("kiki", np.array([0.1, 0.1, 0.1]), 4)
    # print(db.instances_detected_counter)
    # db.add("kiki", np.array([0.1, 0.1, 0.1]), 4)
    # print(db.instances_detected_counter)
    # db.add("kiki", -np.array([0.1, 0.1, 0.1]), 4)
    # print(db.instances_detected_counter)
    # lt.checkForObjectsToRemember(4, db)
    # print(lt.instances_in_memory)
    # db.add("kiki", np.array([0.1, 0.1, 0.1]), 4)
    # print(db.instances_detected_counter)
    # db.add("kiki", np.array([0.1, 0.1, 0.1]), 4)
    # print(db.instances_detected_counter)
    # db.add("kiki", -np.array([0.1, 0.1, 0.1]), 4)
    # print(db.instances_detected_counter)
    # lt.checkForObjectsToRemember(4, db)
    # print(lt.instances_in_memory)
    # print(f"Time: {time.time()-start}")
    # for instance in lt:
    #     print(instance)

    rospy.init_node("memory_node")
    memory_node = MemoryNode()
    rospy.spin()
