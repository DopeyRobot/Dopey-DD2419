#!/usr/bin/env python3
import rospy
import random
import numpy as np
import tf_conversions
from tf2_ros import Buffer, TransformListener, TransformStamped
import tf2_geometry_msgs
import matplotlib.pyplot as plt
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
from typing import List, Tuple
from bounding_box_detection.srv import twoStrInPoseOut, twoStrInPoseOutRequest, closestObj, closestObjRequest
from std_msgs.msg import Bool, Empty, String


class RRTNode:
    def __init__(self, x=None, y=None, parent=None) -> None:
        self.x = x
        self.y = y
        self.parent = parent

        # self.buffer = Buffer(rospy.Duration(100.0))
        # #rospy.sleep(5.0)
        # self.listener = TransformListener(self.buffer)
        #rospy.sleep(2.0)

        self.getPose_client = rospy.ServiceProxy("/get_object_pose", twoStrInPoseOut)
        rospy.wait_for_service("/get_object_pose", timeout=10)

    def set_parent(self, parent: "RRTNode"):
        self.parent = parent

    def get_parent(self):
        return self.parent
    
    def get_start(self):    
        req = twoStrInPoseOutRequest()
        req.str1 = String("map")  # frame_id
        req.str2 = String("base_link")  # object class ball/plushie/box
        desPose = self.getPose_client(req).pose 
        # print(desPose)
        return desPose
        # base_link_origin = PoseStamped()
        # base_link_origin.header.stamp = rospy.Time.now()
        #if self.buffer.can_transform("map", "base_link", base_link_origin.header.stamp, rospy.Duration(1)):
            # transform_to_map = self.buffer.lookup_transform("map", "base_link", base_link_origin.header.stamp, rospy.Duration(1))  
            # baseInMapPose = tf2_geometry_msgs.do_transform_pose(base_link_origin, transform_to_map)

        #     return baseInMapPose
        # else:
        #     return None

class RRTPlanner:
    def __init__(self, start=None, goal=None, num_iterations=100, step_size=2, n_steps=1,runInit=True):
        
        
        # print("clearing variables")
        self.start = None
        self.goal = None #goal

        self.map_data = None
        self.occupancy_grid = None

        self.num_iterations = num_iterations
        self.step_size = step_size
        self.n_steps = n_steps
        self.goal_sample_prob = 0.1

        self.pub_path = rospy.Publisher("/path_topic", Path, queue_size=10,latch=True)
        self.sub_goal = rospy.Subscriber("/send_goal", PoseStamped, self.send_goal_callback)
        self.sub_map = rospy.Subscriber('/occupancygrid', OccupancyGrid, self.get_map_callback)
        self.rate = rospy.Rate(1)

        # self.cur_obj_subscriber = rospy.Subscriber(
        #     "/current_obj_id", String, self.current_object_callback
        # )


        self.path_msg = Path()
        self.path_msg.header.stamp = rospy.Time.now()
        self.path_msg.header.frame_id = "map"

        self.fig, self.ax = plt.subplots()
        
        self.start = RRTNode() 
        start_pose = self.start.get_start()
        while start_pose is None:
            start_pose = self.start.get_start()
        self.start.x = start_pose.pose.position.x
        self.start.y = start_pose.pose.position.y
        
        self.RRT: List[RRTNode] = [self.start]

        self.ready4path = False
        # self.moving2landmark = False
        # self.genLastNode = False

        # self.goalReceivedTicker = 0
        # self.goalProcessedTicker = 0
        if runInit:
            self.run()

    # def current_object_callback(self, msg):
    #     # if "landmark" in msg.data:
    #     #     self.moving2landmark = True
    #     # else:
    #     #     self.moving2landmark = False
    #     self.moving2landmark = True


    def get_map_callback(self, msg):
        if msg is None:
            print(msg)
        self.map_data = msg
        self.occupancy_grid = np.asarray(self.map_data.data, dtype=np.int8).reshape(self.map_data.info.height, self.map_data.info.width)

    def send_goal_callback(self, msg):
        msgGoal = [msg.pose.position.x, msg.pose.position.y]
        # self.goal = [msg.pose.position.x, msg.pose.position.y]
     

        if not np.allclose(np.array(self.goal,dtype=float),np.array(msgGoal,dtype=float)) or self.goal is None:#self.goal != msgGoal:
            # try:
            print("new goal")
            self.goal = msgGoal
            self.ready4path = True
            # self.goalReceivedTicker += 1
            # except:
            #     print("error")
            

    def sample_random(self) -> Tuple[int]:
        if random.random() > self.goal_sample_prob:
            return (
                np.random.uniform(0, self.occupancy_grid.shape[0]),
                np.random.uniform(0, self.occupancy_grid.shape[1]),
            )
        else:
            return self.goal

    def find_nearest(self, x, y) -> RRTNode:
        nearest_node = self.RRT[0]
        for node in self.RRT:
            if np.linalg.norm(
                np.array([x, y]) - np.array([node.x, node.y])
            ) < np.linalg.norm(
                np.array([x, y]) - np.array([nearest_node.x, nearest_node.y])
            ):
                nearest_node = node
        return nearest_node

    def move_step(self, from_x, from_y, to_x, to_y) -> tuple:
        dir = np.array([to_x - from_x, to_y - from_y])
        dir = dir / np.linalg.norm(dir)
        new_pos = np.array([from_x, from_y])
        new_pos = new_pos + self.step_size * dir

        return new_pos[0], new_pos[1]

    def check_map(self, x, y) -> bool:
        # Check bounds   
        if x < self.map_data.info.origin.position.x or y < self.map_data.info.origin.position.y:
            return False
        if x >= self.map_data.info.origin.position.x + self.map_data.info.width * self.map_data.info.resolution or y >= self.map_data.info.origin.position.y + self.map_data.info.height * self.map_data.info.resolution:
            return False

        # Convert the (x,y) coordinate to a grid cell index
        col = int((x - self.map_data.info.origin.position.x) / self.map_data.info.resolution)
        row = int((y - self.map_data.info.origin.position.y) / self.map_data.info.resolution)

        value = self.occupancy_grid[row][col]
        # if self.genLastNode and self.moving2landmark:
        #     return True
        # else:
        if value == 0:
            # Free
            return True
        elif value > 0:
            print("Ran into obstacle")
            # Obstacle
            return False
        else:
            return False#True
            # For now, its unknown..

    def RRT_step(self, nearest_node: RRTNode, target_x, target_y):
        new_node = RRTNode(nearest_node.x, nearest_node.y, nearest_node)
        for step in range(self.n_steps):
            new_x, new_y = self.move_step(new_node.x, new_node.y, target_x, target_y)
            if self.check_map(new_x, new_y):
                new_node.x = new_x
                new_node.y = new_y
            else:
                # print("Ran into obstacle")
                return None

        return new_node

    def generate_RRT(self):
        for i in range(self.num_iterations):
            x_rand, y_rand = self.sample_random()
            nearest_node = self.find_nearest(x_rand, y_rand)
            new_node = self.RRT_step(nearest_node, x_rand, y_rand)
            if new_node is not None:
                self.RRT.append(new_node)
                if (
                    np.linalg.norm(
                        np.array([new_node.x, new_node.y]) - np.array(self.goal)
                    )
                    <= self.step_size
                ):
                    print("Found goal!")
                    goal_node = RRTNode(self.goal[0], self.goal[1], new_node)
                    self.RRT.append(goal_node)
                    break
                # elif (
                #     np.linalg.norm(
                #         np.array([new_node.x, new_node.y]) - np.array(self.goal)
                #     )
                #     <= self.step_size*5
                # ):
                #     self.genLastNode = True
                # else:
                #     self.genLastNode = False


    def generate_path(self):
        current_node = self.RRT[-1]
        while current_node.parent is not None:
            pose_stamped = PoseStamped()
            pose_stamped.header = self.path_msg.header
            pose_stamped.pose.position.x = current_node.x
            pose_stamped.pose.position.y = current_node.y
            pose_stamped.pose.position.z = 0.0
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = 0.0
            pose_stamped.pose.orientation.w = 1.0
            self.path_msg.poses.append(pose_stamped)
            current_node = current_node.get_parent()
        #add start back :)
        pose_stamped = PoseStamped()
        pose_stamped.header = self.path_msg.header
        pose_stamped.pose.position.x = current_node.x
        pose_stamped.pose.position.y = current_node.y
        pose_stamped.pose.position.z = 0.0
        pose_stamped.pose.orientation.x = 0.0
        pose_stamped.pose.orientation.y = 0.0
        pose_stamped.pose.orientation.z = 0.0
        pose_stamped.pose.orientation.w = 1.0
        self.path_msg.poses.append(pose_stamped)

        self.path_msg.poses = self.path_msg.poses[::-1]
        
        for i, pose in enumerate(self.path_msg.poses):
            next_pose = None
            try:
                next_pose = self.path_msg.poses[i + 1]
            except:
                pass
            if next_pose is not None:
                dy = next_pose.pose.position.y - pose.pose.position.y
                dx = next_pose.pose.position.x - pose.pose.position.x

                orientation = np.arctan2(dy, dx)
                quaternian = tf_conversions.transformations.quaternion_from_euler(0,0,orientation)

                pose.pose.orientation.w = quaternian[3]
                pose.pose.orientation.x = quaternian[0]
                pose.pose.orientation.y = quaternian[1]
                pose.pose.orientation.z = quaternian[2]

    def publish_path(self):
        self.pub_path.publish(self.path_msg)
        
    
    def plot_RRT_tree(self):
        self.fig, self.ax = plt.subplots()
        self.ax.scatter(self.goal[0], self.goal[1], color="red", s=10)
        for i, node in enumerate(self.RRT):

            self.ax.scatter(node.x, node.y, color="blue", s=10)

            parent = node.parent
            if parent is None:
                continue

            self.ax.plot(
                [node.x, parent.x], [node.y, parent.y], color="blue", alpha=0.3
            )

        plt.show()

    def run(self):
        while not rospy.is_shutdown():
            #planner = RRTPlanner(start, goal, num_iterations=1000, step_size=0.3)
            #print("goal not received")
            #print(self.goal)
            # print("R: ",self.goalReceivedTicker)
            # print("P: ", self.goalProcessedTicker)
            if self.ready4path:#self.goal is not None and self.goalReceivedTicker != self.goalProcessedTicker and self.ready4path:
                
                self.__init__(num_iterations=self.num_iterations,step_size=self.step_size,runInit=False)
                #TODO: after successfully arriving atfirst goal, the secodn goal always has teh first noed in the origin of odom and not base_link. Fix this. 
                # self.ready4path = False
                # self.goalProcessedTicker += 1
                # self.start = RRTNode() 
                # self.start.x = self.start.get_start().pose.position.x
                # self.start.y = self.start.get_start().pose.position.y
                # self.RRT: List[RRTNode] = [self.start]
                #print("goal received")
        
                self.generate_RRT()
                self.generate_path()
                #planner.plot_RRT_tree()
                self.publish_path()
                #self.goal = None


if __name__ == "__main__":
    rospy.init_node("rrt")
    # buffer = Buffer(rospy.Duration(100.0))
    # if buffer.can_transform("map", "base_link", rospy.Time.now(), rospy.Duration(2)):

    RRTPlanner(start=None, goal=None, num_iterations=1000, step_size=0.2)
    # try:
    #     start = [0, 0]
    #     goal = [0, 0]
    #     while not rospy.is_shutdown():
    #         planner = RRTPlanner(start, goal, num_iterations=1000, step_size=0.3)
    #         planner.generate_RRT()
    #         planner.generate_path()
    #         #planner.plot_RRT_tree()
    #         planner.publish_path()
    # except Exception as e:
    #     print("Passing in RRT due to:")
    #     print(e)
    rospy.spin()
