#!/usr/bin/env python


import py_trees as pt, py_trees_ros as ptr, rospy
from behaviours_student import counter
from reactive_sequence import RSequence

from geometry_msgs.msg import PoseStamped, Pose, PoseWithCovarianceStamped
from std_srvs.srv import Empty, SetBool, SetBoolRequest 

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseFeedback, MoveBaseActionFeedback
from actionlib_msgs.msg import GoalID

import py_trees as pt, py_trees_ros as ptr, rospy
from geometry_msgs.msg import Twist
from actionlib import SimpleActionClient
from play_motion_msgs.msg import PlayMotionAction, PlayMotionGoal
from robotics_project.srv import MoveHead, MoveHeadRequest, MoveHeadResponse

from gazebo_msgs.msg import ModelState
from gazebo_msgs.srv import SetModelState

import functools

import pdb

import std_msgs

hasMoved2Table2 = False
failedPlace = False


class BehaviourTree(ptr.trees.BehaviourTree):

	def __init__(self):

		rospy.loginfo("Initialising behaviour tree")

		# # go to door until at door
		# b0 = pt.composites.Selector(
		# 	name="Go to door fallback", 
		# 	children=[counter(30, "At door?"), go("Go to door!", 1, 0)]
		# )

		# # tuck the arm
		# b1 = tuckarm()

		# # go to table
		# b2 = pt.composites.Selector(
		# 	name="Go to table fallback",
		# 	children=[counter(5, "At table?"), go("Go to table!", 0, -1)]
		# )

		# # move to chair
		# b3 = pt.composites.Selector(
		# 	name="Go to chair fallback",
		# 	children=[counter(13, "At chair?"), go("Go to chair!", 1, 0)]
		# )

		# # lower head
		# b4 = movehead("down")

		# # become the tree
		# tree = RSequence(name="Main sequence", children=[b0, b1, b2, b3, b4])
		
		####### C implementation
		# btuck = tuckarm()
		# bhead = movehead("down")

		# #tree = RSequence(name="Main sequence", children=[btuck,bhead])
		
		# # bDetectCube1 = pt.composites.Selector(
		# # 	name="Detect cube fallback 1", 
		# # 	children=[counter(30, "Cube detected on table 1"), CubeDetector()]
		# # )

		# bDetectCube1 = CubeDetector()

		# #tree = RSequence(name="Main sequence", children=[tree,bDetectCube1])

		# tree = RSequence(name="Main sequence", children=[bhead,bDetectCube1])
		# tree = RSequence(name="Main sequence", children=[tree,btuck])

		# bPickCube = PickCube()

		# tree = RSequence(name="Main sequence", children=[tree,bPickCube])

		# bMove2table2 = pt.composites.Selector(
		# 	name="Move to table 2", 
		# 	children=[counter(37, "At table 2"), goCustom("Going to table 2", 1, 1)]
		# )

		# tree = RSequence(name="Main sequence", children=[tree,bMove2table2])

		# bPlaceCube = PlaceCube()

		# tree = RSequence(name="Main sequence", children=[tree,bPlaceCube])

		# bDetectCubeOrGo2init = pt.composites.Selector(
		# 	name="Detect cube fallback 2 or go back to ", 
		# 	children=[CubeDetector2(),counter(37, "At table 1"),goCustom2("Going back to init state", 1, -1)]
		# )

		# # bGo2init = pt.composites.Selector(
		# # 	name="Go back to init state fallback", 
		# # 	children=[counterCustom(30, "At init state"), go("Going back to init state", 1, 0)]
		# # )

		# # bFinal = pt.composites.Selector(
		# # 	name="Cube on table 2 or go back to init state ", 
		# # 	children=[bDetectCube2, bGo2init]
		# # )

		# tree = RSequence(name="Main sequence", children=[tree,bDetectCubeOrGo2init])

		# A-implementation 

		
		#bNavigate2Pick = Navigate("pick")

		#tree = RSequence(name="Main sequence", children=[bNavigate2Pick,bDetectCubeA1])
		#tree = RSequence(name="Main sequence", children=[btuck,bNavigate2Pick])
		# tree = RSequence(name="Main sequence", children=[tree,bPickCube])

		# tree = RSequence(name="Main sequence", children=[tree,bNavigate2Place])

		# tree = RSequence(name="Main sequence", children=[tree,bPlaceCube])

		# tree = RSequence(name="Main sequence", children=[tree,bNavigate2Place])

		# #tree = RSequence(name="Main sequence", children=[tree,bDetectCube])

		#bLocalize = Localize()

		#tree = RSequence(name="Main sequence", children=[bLocalize,tree,bDetectCubeA2])

		
		btuck = tuckarm()

		bheadup = movehead("up")

		tree = RSequence(name="Main sequence", children=[btuck,bheadup])

		bLocNavPick = LocNav("pick")

		tree = RSequence(name="Main sequence", children=[tree,bLocNavPick])

		#tree = RSequence(name="Main sequence", children=[tree,bheaddown])

		bheaddown = movehead("down")

		tree = RSequence(name="Main sequence", children=[tree,bheaddown])

		bDetect1 = CubeDetector()

		tree = RSequence(name="Main sequence", children=[tree,bDetect1])

		bpick = PickCube()

		tree = RSequence(name="Main sequence", children=[tree,bpick])

		bheadup2 = movehead("up")

		tree = RSequence(name="Main sequence", children=[tree,bheadup2])

		bLocNavPlace = LocNav("place")

		tree = RSequence(name="Main sequence", children=[tree,bLocNavPlace])

		bplace = PlaceCube()

		tree = RSequence(name="Main sequence", children=[tree,bplace])

		bheaddown2 = movehead("down")

		tree = RSequence(name="Main sequence", children=[tree,bheaddown2])

		#bDetect2 = CubeDetector2()
		#bReset = Resetter()

		bFinal = pt.composites.Selector(
			name="Sucessful placement on table 2 or reset tree ", 
			children=[CubeDetector2(),ResetCounter(50,"Resetting nodes in tree"), Resetter()]
		)

		tree = RSequence(name="Main sequence", children=[tree,bFinal])

		# bheadup3 = movehead("up")

		# tree = RSequence(name="Main sequence", children=[tree,bheadup3])

		##TODO reinit cube at table 1 if detector 2 fails

		#self.tree = tree
		super(BehaviourTree, self).__init__(tree)

		#self.post_tick_handlers = [self.reInitNodes()]
		# snapshot_visitor = pt.visitors.SnapshotVisitor()
		# self.add_post_tick_handler(functools.partial(self.reInitNodes, snapshot_visitor))
		# self.visitors.append(snapshot_visitor)

		#self.add_post_tick_handler(reInitNodes)
		
		#self.last_tree.behaviours
		#rospy.loginfo(tree.children[1].p)
		# execute the behaviour tree
		rospy.sleep(5)
		self.setup(timeout=10000)
		while not rospy.is_shutdown(): self.tick_tock(1)	

# def reInitNodes(BT):
# 	global failedPlace
# 	if failedPlace:
		
# 		# for behaviour in BT.last_tree.behaviours:#.iterate():
# 		# 	#for node in behaviour:#.iterate():
# 		# 	rospy.loginfo(behaviour.name)
# 		# 	rospy.loginfo(behaviour.status)
# 		#BT.root.children.reInit()
# 		#pdb.set_trace()
# 		BT.tick()
# 			#behaviour.status#initialise()
		

class CubeDetector(pt.behaviour.Behaviour):

	"""
	Detects cube in camera frame(s)
	"""

	def __init__(self):
		#rospy.loginfo("Initialising move head behaviour.")
		self.wait1time = True
		# server
		self.cube_detect_top = rospy.get_param(rospy.get_name() + '/cube_pose')
		self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
		#self.robot_frame_top = rospy.get_param(rospy.get_name() + '/robot_base_frame')
		#self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
		self.msg = False
		self.cube_detect_subs = rospy.Subscriber(self.cube_detect_top, PoseStamped, self.msg_received)

		self.aruco_detect_pub = rospy.Publisher(self.aruco_pose_top, PoseStamped, queue_size=10)
		#self.robot_frame_pub = rospy.Publisher(self.robot_frame_top, std_msgs.msg.String, queue_size=10)
		#self.aruco_detect_pub.publish(self.msg)
		self.newmsg=False
		self.cnt = 0
		self.cntMAX = 30
		self.printed = False

		# become a behaviour
		super(CubeDetector, self).__init__("Detecting Cube!")

	def msg_received(self,msg):
		if self.newmsg:
			self.msg = msg
	
	# def initialise(self):
	# 	self-return super().initialise()

	def update(self):
		global failedPlace
		if failedPlace:
			self.wait1time = True
			self.msg = False
			self.newmsg=False
			self.cnt = 0
			self.cntMAX = 30
			self.printed = False
			#rospy.loginfo(failedPlace)
			return pt.common.Status.SUCCESS

		# rate = rospy.Rate(1)
		# self.robot_frame_top.publish("/base_link")
		# rate.sleep()


		if self.wait1time:
			rospy.sleep(5)
			self.wait1time = False
		self.newmsg = True
		self.cnt += 1
		# # send the message
		# if self.msg is not None:
		# 	rate = rospy.Rate(10)
		# 	self.aruco_detect_pub.publish(self.msg)
		# 	rate.sleep()

		# # tell the tree that you're running
		# return pt.common.Status.RUNNING

		# success if done
		#pdb.set_trace()
		rospy.loginfo(self.cnt)

		if self.msg:
			if not self.printed:
				rospy.loginfo("Cube detected")
				self.printed = True
				#rospy.loginfo(self.msg)
			self.aruco_detect_pub.publish(self.msg)
			return pt.common.Status.SUCCESS

		elif self.cnt < self.cntMAX:
			rospy.loginfo("Detecting cube")
			return pt.common.Status.RUNNING
		
		else:#if self.cnt == self.cntMAX:
			if not self.printed: 
				rospy.loginfo("Cube not detected")
				#self.msg = False
				#rospy.sleep(1)
				self.printed = False
			return pt.common.Status.FAILURE

class PickCube(pt.behaviour.Behaviour):

	"""
	Detects cube in camera frame(s)
	"""

	def __init__(self):
		rospy.loginfo("Initialising pick cube behaviour.")

		# server
		self.pick_srv_nm = rospy.get_param(rospy.get_name() + '/pick_srv')
		self.pick_srv = rospy.ServiceProxy(self.pick_srv_nm, SetBool)
		self.wait1time = True
		rospy.wait_for_service(self.pick_srv_nm, timeout=30)

		# execution checker
		self.tried = False
		self.done = False

		# become a behaviour
		super(PickCube, self).__init__("Picking Cube!")

	def update(self):
		global failedPlace
		if failedPlace:
			self.tried = False
			self.done = False
			return pt.common.Status.SUCCESS

		if self.wait1time:
			rospy.sleep(5)
			self.wait1time = False
		# success if done
		if self.done:
			return pt.common.Status.SUCCESS

		# try if not tried
		elif not self.tried:

			# command
			self.pick_req = self.pick_srv(True)
			self.tried = True

			# tell the tree you're running
			return pt.common.Status.RUNNING

		# if succesful
		elif self.pick_req.success:
			self.done = True
			return pt.common.Status.SUCCESS

		# if failed
		elif not self.pick_req.success:
			return pt.common.Status.FAILURE

		# if still trying
		else:
			return pt.common.Status.RUNNING

class goCustom(pt.behaviour.Behaviour):

	"""
	Returns running and commands a velocity indefinitely.
	"""

	def __init__(self, name, linear, angular):
		global hasMoved2Table2
		hasMoved2Table2 = True

		rospy.loginfo("Initialising goCustom behaviour.")

		# action space
		#self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
		self.cmd_vel_top = "/key_vel"
		#rospy.loginfo(self.cmd_vel_top)
		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

		# command
		self.move_msg = Twist()

		self.cnt = 0
		self.linear = linear
		self.angular = angular
		
		# become a behaviour
		super(goCustom, self).__init__(name)

	def update(self):
		
		# send the message
		rate = rospy.Rate(10)
		
		rate.sleep()

		rotTime = 28
		if self.cnt < rotTime:
			self.move_msg.linear.x = 0
			self.move_msg.angular.z = self.angular
		elif self.cnt == rotTime:
			self.move_msg.linear.x = 0
			self.move_msg.angular.z = 0
			rospy.sleep(2)
		else:
			self.move_msg.linear.x = self.linear
			self.move_msg.angular.z = 0

		self.cnt += 1
		self.cmd_vel_pub.publish(self.move_msg)
		# tell the tree that you're running
		return pt.common.Status.RUNNING

class PlaceCube(pt.behaviour.Behaviour):

	"""
	Detects cube in camera frame(s)
	"""

	def __init__(self):
		rospy.loginfo("Initialising place cube behaviour.")

		# server
		self.place_srv_nm = rospy.get_param(rospy.get_name() + '/place_srv')
		self.place_srv = rospy.ServiceProxy(self.place_srv_nm, SetBool)
		rospy.wait_for_service(self.place_srv_nm, timeout=30)

		# execution checker
		self.tried = False
		self.done = False
		self.wait1time=False

		# become a behaviour
		super(PlaceCube, self).__init__("Place Cube!")

	def update(self):
		global failedPlace
		if failedPlace:
			self.tried = False
			self.done = False
			self.wait1time=False
			return pt.common.Status.SUCCESS

		if not self.wait1time:
			rospy.sleep(2)
			self.wait1time=True
		# success if done
		if self.done:
			return pt.common.Status.SUCCESS

		# try if not tried
		elif not self.tried:

			# command
			self.place_srv_req = self.place_srv(True)
			self.tried = True

			# tell the tree you're running
			return pt.common.Status.RUNNING

		# if succesful
		elif self.place_srv_req.success:
			self.done = True
			return pt.common.Status.SUCCESS

		# if failed
		elif not self.place_srv_req.success:
			return pt.common.Status.FAILURE

		# if still trying
		else:
			return pt.common.Status.RUNNING

class goCustom2(pt.behaviour.Behaviour):

	"""
	Returns running and commands a velocity indefinitely.
	"""

	def __init__(self, name, linear, angular):
		global hasMoved2Table2

		rospy.loginfo("Initialising goCustom behaviour.")

		# action space
		#self.cmd_vel_top = rospy.get_param(rospy.get_name() + '/cmd_vel_topic')
		self.cmd_vel_top = "/key_vel"
		#rospy.loginfo(self.cmd_vel_top)
		self.cmd_vel_pub = rospy.Publisher(self.cmd_vel_top, Twist, queue_size=10)

		# command
		self.move_msg = Twist()

		self.cnt = 0
		self.linear = linear
		self.angular = angular
		
		# become a behaviour
		super(goCustom2, self).__init__(name)

	def update(self):
		
		# send the message
		rate = rospy.Rate(10)
		
		rate.sleep()

		rotTime = 28
		if hasMoved2Table2:
			if self.cnt < rotTime:
				self.move_msg.linear.x = 0
				self.move_msg.angular.z = self.angular
			elif self.cnt == rotTime:
				self.move_msg.linear.x = 0
				self.move_msg.angular.z = 0
				rospy.sleep(2)
			else:
				self.move_msg.linear.x = self.linear
				self.move_msg.angular.z = 0

			self.cnt += 1
			self.cmd_vel_pub.publish(self.move_msg)
			# tell the tree that you're running
		return pt.common.Status.RUNNING
class CubeDetector2(pt.behaviour.Behaviour):

	"""
	Detects cube in camera frame(s)
	"""

	def __init__(self):
		#rospy.loginfo("Initialising move head behaviour.")
		self.wait1time = True
		# server
		self.cube_detect_top = rospy.get_param(rospy.get_name() + '/cube_pose')
		#self.aruco_pose_top = rospy.get_param(rospy.get_name() + '/aruco_pose_topic')
		#self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
		self.msg = False
		self.cube_detect_subs = rospy.Subscriber(self.cube_detect_top, PoseStamped, self.msg_received)

		#self.aruco_detect_pub = rospy.Publisher(self.aruco_pose_top, PoseStamped, queue_size=10)
		#self.aruco_detect_pub.publish(self.msg)

		self.cnt = 0
		self.cntMAX = 30
		self.printed = False
		self.newmsg=False

		# become a behaviour
		super(CubeDetector2, self).__init__("Detecting Cube!")

	def msg_received(self,msg):
		if self.newmsg:
			self.msg = msg

	def reInit():
		rospy.loginfo("inside reinit of cubedetect2")

	def update(self):
		global failedPlace
		if failedPlace:
			self.wait1time = True
			
			self.msg = False

			self.cnt = 0
			self.cntMAX = 30
			self.printed = False
			self.newmsg=False
			return pt.common.Status.FAILURE

		#global failedPlace
		if self.wait1time:
			rospy.sleep(5)
			self.wait1time = False

		self.newmsg=True
		self.cnt += 1
		# # send the message
		# if self.msg is not None:
		# 	rate = rospy.Rate(10)
		# 	self.aruco_detect_pub.publish(self.msg)
		# 	rate.sleep()

		# # tell the tree that you're running
		# return pt.common.Status.RUNNING

		# success if done
		if self.msg:
			if not self.printed:
				rospy.loginfo("Cube detected")
				self.printed = True
				rospy.loginfo(self.msg)
			return pt.common.Status.SUCCESS

		elif self.cnt < self.cntMAX:
			rospy.loginfo("Detecting cube")
			return pt.common.Status.RUNNING
		
		else:#if self.cnt == self.cntMAX:
			if not self.printed: 
				rospy.loginfo("Cube not detected")
				self.printed = True
				failedPlace = True
			return pt.common.Status.FAILURE

class Localize(pt.behaviour.Behaviour):

	"""
	Detects cube in camera frame(s)
	"""

	def __init__(self):
		#Get amcl_pose param 
		#self.amcl_pose_top = rospy.get_param(rospy.get_name() + '/cube_pose')
		self.amcl_pose_subs = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_received)
		self.global_loc_srv = rospy.ServiceProxy("/global_localization", Empty) #create client
		self.update_srv = rospy.ServiceProxy("/request_nomotion_update",Empty)
		rospy.wait_for_service("/global_localization", timeout=30) #wait for service name
		rospy.wait_for_service("/request_nomotion_update", timeout=30)
		
		self.global_loc_srv()
		
		self.initDistr = False
		# Create client for global_loc serv
		# Call client
		self.poseWithCovariance = False

		# become a behaviour
		super(Localize, self).__init__("Localizing!")
	
	def pose_received(self,poseWithCovariance):
		self.poseWithCovariance = poseWithCovariance

	def update(self):
		
		converged = True
		if self.poseWithCovariance:
			covariance = self.poseWithCovariance.pose.covariance
			for i in range(6):
				for j in range(6):
					if i != j:
						if abs(covariance[i*6+j]) > 10**(-5):
							converged = False
							self.update_srv()
							break
		
			#rospy.loginfo(self.poseWithCovariance.pose.pose)						
		# success if done
		if self.poseWithCovariance and converged:
			#rospy.loginfo(self.poseWithCovariance.pose.pose)
			rospy.loginfo("Localization done!")
			return pt.common.Status.SUCCESS

		else:
			#rospy.loginfo("Im here")
			return pt.common.Status.RUNNING

class Navigate(pt.behaviour.Behaviour):

	"""
	Detects cube in camera frame(s)
	"""

	def __init__(self,action):
		#subsciber to pick pose topic
		#subscriber to place pose topic
		self.action = action
		self.pick_pose = False
		self.place_pose = False
		self.cmd_vel = False
		#self.init = False

		self.pick_pose_top = rospy.get_param(rospy.get_name()+"/pick_pose_topic")
		self.place_pose_top = rospy.get_param(rospy.get_name()+"/place_pose_topic")
		#self.nav_goal_top = rospy.get_param(rospy.get_name()+"/nav_goal_topic")
		#self.cmd_vel_top = rospy.get_param(rospy.get_name()+"/cmd_vel_topic")
		self.pick_pose_subs = rospy.Subscriber(self.pick_pose_top, PoseStamped, self.pickMsg)
		self.place_pose_subs = rospy.Subscriber(self.place_pose_top, PoseStamped, self.placeMsg)
		self.cmd_vel_subs = rospy.Subscriber("/nav_vel",Twist,self.cmdMsg)

		self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped)
		#self.goal_pub = rospy.Publisher(self.nav_goal_top, PoseStamped, queue_size=10)
		#self.cmd_vel_pub = rospy.Publisher("/key_vel", Twist, queue_size=10)

		# #
		# self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
		# self.client.wait_for_server()

		# goal = MoveBaseGoal()
		# # Fill in the goal here
		
		# goal.target_pose.header.frame_id = "odom"
		# goal.target_pose.header.stamp = rospy.Time.now()
		# goal.target_pose.pose.position.x = 0
		# goal.target_pose.pose.orientation.w = 0

		# self.client.send_goal(goal)
		# self.wait = self.client.wait_for_result()


		
		# become a behaviour
		super(Navigate, self).__init__("Navigating!")
	
	def pickMsg(self,pose):
		self.pick_pose = pose
	
	def placeMsg(self,pose):
		self.place_pose = pose

	def cmdMsg(self,twist):
		self.cmd_vel = twist

	def update(self):
		#if not self.init:
		rate = rospy.Rate(10)

		rate.sleep()

		if self.cmd_vel:
			x = self.cmd_vel.linear.x
			y = self.cmd_vel.linear.y
			w = self.cmd_vel.angular.z
			#tol = 10**(-5)
			#if abs(x) < tol and abs(y) < tol and abs(w) < tol:
			if x == 0 and y == 0 and w == 0:
				rospy.loginfo(self.cmd_vel)
				return pt.common.Status.SUCCESS
			else:
				if self.action == "pick":
					self.goal_pub.publish(self.pick_pose)
				elif self.action == "place":
					self.goal_pub.publish(self.place_pose)
				return pt.common.Status.RUNNING

		else:
			if self.action == "pick":
				self.goal_pub.publish(self.pick_pose)
			elif self.action == "place":
				self.goal_pub.publish(self.place_pose)
			return pt.common.Status.RUNNING

class LocNav(pt.behaviour.Behaviour):

	"""
	Detects cube in camera frame(s)
	"""

	def __init__(self,action):
		self.startLoc = True
		self.cntMax = 400
		self.cnt = self.cntMax
		
		#Localization
		self.amcl_pose_subs = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.pose_received)
		self.global_loc_srv = rospy.ServiceProxy("/global_localization", Empty) #create client
		self.clear_cost_map_srv = rospy.ServiceProxy("/move_base/clear_costmaps", Empty) #create client
		self.update_srv = rospy.ServiceProxy("/request_nomotion_update",Empty)
		rospy.wait_for_service("/global_localization", timeout=30) #wait for service name
		rospy.wait_for_service("/request_nomotion_update", timeout=30)
		rospy.wait_for_service("/move_base/clear_costmaps", timeout=30)
		
		
		
		self.initDistr = False
		# Create client for global_loc serv
		# Call client
		self.poseWithCovariance = False

		#Navigation
		self.action = action
		self.pick_pose = False
		self.place_pose = False
		self.cmd_vel = False
		self.notStuckCnt = 0
		#self.init = False
		


		self.pick_pose_top = rospy.get_param(rospy.get_name()+"/pick_pose_topic")
		self.place_pose_top = rospy.get_param(rospy.get_name()+"/place_pose_topic")
		#self.nav_goal_top = rospy.get_param(rospy.get_name()+"/nav_goal_topic")
		#self.cmd_vel_top = rospy.get_param(rospy.get_name()+"/cmd_vel_topic")
		self.pick_pose_subs = rospy.Subscriber(self.pick_pose_top, PoseStamped, self.pickMsg)
		self.place_pose_subs = rospy.Subscriber(self.place_pose_top, PoseStamped, self.placeMsg)
		self.cmd_vel_subs = rospy.Subscriber("/nav_vel",Twist,self.cmdMsg)
		self.feedback_subs = rospy.Subscriber("/move_base/feedback", MoveBaseActionFeedback, self.feedbackMsg)
		self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped)
		self.cmd_vel_pub = rospy.Publisher("/nav_vel", Twist, queue_size=10)
		self.move_base_cancel_pub = rospy.Publisher("/move_base/cancel",GoalID)

		## ActionClient
		self.client = actionlib.SimpleActionClient('/move_base',MoveBaseAction)
		self.client.wait_for_server()

		self.goal = MoveBaseGoal()
		self.goal.target_pose.header.frame_id = "map"
		self.goal.target_pose.header.stamp = rospy.Time.now()

		self.feedbackPose = None
		self.isDone = False

		self.cal_msg = Twist()
		self.cal_msg.angular.z = 1


		rospy.sleep(2)
		if self.action == "pick":
			#self.goal_pub.publish(self.pick_pose)
			self.goal_pose = self.pick_pose
			
		elif self.action == "place":
			self.goal_pose = self.place_pose
			#self.goal_pub.publish(self.place_pose)
			
		#rospy.loginfo(self.goal_pose)
		# become a behaviour
		super(LocNav, self).__init__("LocNaving!")
	
	def pose_received(self,poseWithCovariance):
		self.poseWithCovariance = poseWithCovariance
	
	def pickMsg(self,pose):
		self.pick_pose = pose
	
	def placeMsg(self,pose):
		self.place_pose = pose

	def cmdMsg(self,twist):
		self.cmd_vel = twist
	
	def feedbackMsg(self,feedback):
		self.feedbackPose = feedback

	def update(self):
		global failedPlace
		if failedPlace:
			self.startLoc = True
			self.cnt = self.cntMax
			self.initDistr = False
			# Create client for global_loc serv
			# Call client
			self.poseWithCovariance = False

			self.pick_pose = False
			self.place_pose = False
			self.cmd_vel = False
			self.notStuckCnt = 0
			
			self.feedbackPose = None
			self.isDone = False

			self.cal_msg.angular.z = 1

			return pt.common.Status.SUCCESS

		if not self.isDone:
			self.cnt += 1
			
			#cntMax = 1000
			#localize
			if self.cnt >= self.cntMax:
				if self.startLoc:#self.cnt == self.cntMax+1 and self.startLoc:
					self.global_loc_srv()
					rospy.loginfo("Initiating localization")
					self.startLoc = False
				converged = True
				self.cal_msg.angular.z = 1
				self.cmd_vel_pub.publish(self.cal_msg)
				if self.poseWithCovariance:
					covariance = self.poseWithCovariance.pose.covariance
					for i in range(6):
						for j in range(6):
							if i != j:
								if abs(covariance[i*6+j]) > 10**(-3):
									converged = False
									self.update_srv()
									break
				
					#rospy.loginfo(self.poseWithCovariance.pose.pose)						
				# success if done
				if self.poseWithCovariance and converged:
					#rospy.loginfo(self.poseWithCovariance.pose.pose)
					self.cal_msg.angular.z = 0
					self.cmd_vel_pub.publish(self.cal_msg)
					
					rospy.sleep(1)
					#self.clear_cost_map_srv()
					rospy.loginfo("Localization done!")
					rospy.loginfo(self.cnt)
					self.cnt = 0
					self.startLoc = True
					#send goal after localization is done
					# if self.action == "pick":
					# 	self.goal_pub.publish(self.pick_pose)
					# elif self.action == "place":
					# 	self.goal_pub.publish(self.place_pose)

					#ActionLib
					# if self.action == "pick":
					# 	#self.goal_pub.publish(self.pick_pose)
					# 	goal_pose = self.pick_pose
						
					# elif self.action == "place":
					# 	goal_pose = self.place_pose
					# 	#self.goal_pub.publish(self.place_pose)

					
					self.goal.target_pose.pose.position.x = self.goal_pose.pose.position.x
					self.goal.target_pose.pose.position.y = self.goal_pose.pose.position.y
					self.goal.target_pose.pose.position.z = self.goal_pose.pose.position.z
					self.goal.target_pose.pose.orientation.x = self.goal_pose.pose.orientation.x
					self.goal.target_pose.pose.orientation.y = self.goal_pose.pose.orientation.y
					self.goal.target_pose.pose.orientation.z = self.goal_pose.pose.orientation.z
					self.goal.target_pose.pose.orientation.w = self.goal_pose.pose.orientation.w
					
					self.client.send_goal(self.goal)
					#self.clear_cost_map_srv()
					#wait = self.client.wait_for_result()

					# if not wait:
					# 	rospy.logerr("Action server not available!")
					# 	rospy.signal_shutdown("Action server not available!")
					# else:
					# 	return self.client.get_result()

					return pt.common.Status.RUNNING

				else:
					#rospy.loginfo("Im here")
					return pt.common.Status.RUNNING
			#navigate
			else:
				rate = rospy.Rate(10)

				rate.sleep()

				currentPoseEstimate = self.feedbackPose.feedback.base_position.pose

				if self.cmd_vel:
					# x = self.cmd_vel.linear.x
					# y = self.cmd_vel.linear.y
					# w = self.cmd_vel.angular.z
					xc = currentPoseEstimate.position.x
					yc = currentPoseEstimate.position.y
					zc = currentPoseEstimate.position.z
					q1c = currentPoseEstimate.orientation.x
					q2c = currentPoseEstimate.orientation.y
					q3c = currentPoseEstimate.orientation.z
					q4c  = currentPoseEstimate.orientation.w

					xg = self.goal_pose.pose.position.x
					yg = self.goal_pose.pose.position.y
					zg = self.goal_pose.pose.position.z
					q1g = self.goal_pose.pose.orientation.x
					q2g = self.goal_pose.pose.orientation.y
					q3g = self.goal_pose.pose.orientation.z
					q4g = self.goal_pose.pose.orientation.w

					tol = 0.075
					#if abs(x) < tol and abs(y) < tol and abs(w) < tol:
					#if x == 0 and y == 0 and w == 0 and self.cnt > 100:
					if abs(xc-xg) < tol and abs(yc-yg) < tol and abs(zc-zg) < tol and abs(q1c-q1g) < tol and abs(q2c-q2g) < tol and abs(q3c-q3g) < tol and abs(q4c-q4g) < tol and self.cmd_vel.angular.z == 0:
						#self.notStuckCnt += 1
						#if self.notStuckCnt > 100:
						rospy.loginfo("Navigation done!")
						self.clear_cost_map_srv()
						cGoal = GoalID()
						self.move_base_cancel_pub.publish(cGoal)
						self.isDone = True
						rospy.sleep(2)
						return pt.common.Status.SUCCESS
						# else:
						# 	return pt.common.Status.RUNNING
					else:
						# if self.action == "pick":
						# 	self.goal_pub.publish(self.pick_pose)
						# elif self.action == "place":
						# 	self.goal_pub.publish(self.place_pose)
						rospy.loginfo("Reinit localization in:")
						rospy.loginfo(self.cntMax-self.cnt)
						if self.cnt == self.cntMax-1:
							self.clear_cost_map_srv()
						return pt.common.Status.RUNNING

				else:
					# if self.action == "pick":
					# 	self.goal_pub.publish(self.pick_pose)
					# elif self.action == "place":
					# 	self.goal_pub.publish(self.place_pose)
					# self.cnt += 1
					return pt.common.Status.RUNNING
		else:
			return pt.common.Status.SUCCESS

class tuckarm(pt.behaviour.Behaviour):

    """
    Sends a goal to the tuck arm action server.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self):

        rospy.loginfo("Initialising tuck arm behaviour.")

        # Set up action client
        self.play_motion_ac = SimpleActionClient("/play_motion", PlayMotionAction)

        # personal goal setting
        self.goal = PlayMotionGoal()
        self.goal.motion_name = 'home'
        self.goal.skip_planning = True

        # execution checker
        self.sent_goal = False
        self.finished = False

        # become a behaviour
        super(tuckarm, self).__init__("Tuck arm!")

    # def initialise(self):
    #     # self.finished = False
    #     # self.sent_goal = False
    #     rospy.loginfo("reinit")


    def update(self):
        global failedPlace
        if failedPlace:
            self.goal.skip_planning = True

            # execution checker
            self.sent_goal = False
            self.finished = False
            return pt.common.Status.SUCCESS

        # already tucked the arm
        if self.finished: 
            #rospy.loginfo("arm tucked")
            return pt.common.Status.SUCCESS
        
        # command to tuck arm if haven't already
        elif not self.sent_goal:

            # send the goal
            self.play_motion_ac.send_goal(self.goal)
            self.sent_goal = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if I was succesful! :)))))))))
        elif self.play_motion_ac.get_result():

            # than I'm finished!
            self.finished = True
            #rospy.loginfo("arm tucked")
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.play_motion_ac.get_result():
            return pt.common.Status.FAILURE

        # if I'm still trying :|
        else:
            return pt.common.Status.RUNNING
    
    # ## Custom
    # def initialise(self):
    #     return super().initialise()

class movehead(pt.behaviour.Behaviour):

    """
    Lowers or raisesthe head of the robot.
    Returns running whilst awaiting the result,
    success if the action was succesful, and v.v..
    """

    def __init__(self, direction):

        rospy.loginfo("Initialising move head behaviour.")

        # server
        mv_head_srv_nm = rospy.get_param(rospy.get_name() + '/move_head_srv')
        self.move_head_srv = rospy.ServiceProxy(mv_head_srv_nm, MoveHead)
        rospy.wait_for_service(mv_head_srv_nm, timeout=30)

        # head movement direction; "down" or "up"
        self.direction = direction

        # execution checker
        self.tried = False
        self.done = False

        # become a behaviour
        super(movehead, self).__init__("Lower head!")
    
    def reInit():
       rospy.loginfo("inside reinit of movehead")
    


    def update(self):
        global failedPlace
        if failedPlace:
            self.tried = False
            self.done = False
            return pt.common.Status.SUCCESS
        
        # success if done
        if self.done:
            return pt.common.Status.SUCCESS

        # try if not tried
        elif not self.tried:

            # command
            self.move_head_req = self.move_head_srv(self.direction)
            self.tried = True

            # tell the tree you're running
            return pt.common.Status.RUNNING

        # if succesful
        elif self.move_head_req.success:
            self.done = True
            return pt.common.Status.SUCCESS

        # if failed
        elif not self.move_head_req.success:
            return pt.common.Status.FAILURE

        # if still trying
        else:
            return pt.common.Status.RUNNING

class Resetter(pt.behaviour.Behaviour):
	"""
	Resets tree
	"""

	def __init__(self):
		# rospy.loginfo("Init Tree Resetter")
		# self.cube_reset_srv = rospy.ServiceProxy("/gazebo/set_model_state", Empty) #create client
		# rospy.wait_for_service("/gazebo/set_model_state", timeout=30) #wait for service name

		#self.cnt = 1

		# become a behaviour
		super(Resetter, self).__init__("Place Cube!")

	def update(self):
		# global failedPlace
		# if failedPlace:
		# 	self.cnt+=1
	
		# 	if self.cnt%50==0:
		# 		failedPlace =False
		# 		rospy.loginfo("Tree resetter done")
		# 		self.cube_reset_srv()
		# 		return pt.common.Status.SUCCESS
		# 	else:
		# 		rospy.loginfo("Currently resetting, counter at: ")
		# 		rospy.loginfo(self.cnt)
		# 		return pt.common.Status.FAILURE
		# else:
		# 	rospy.loginfo("failedPlace is False")
		# 	return pt.common.Status.FAILURE
		rospy.loginfo("Currently resetting")
		return pt.common.Status.RUNNING

class ResetCounter(pt.behaviour.Behaviour):

	"""
	Returns running for n ticks and success thereafter.
	"""

	def __init__(self, n, name):
		rospy.loginfo("Init Tree Resetter")
		self.cube_reset_srv = rospy.ServiceProxy("/gazebo/set_model_state", SetModelState) #create client
		rospy.wait_for_service("/gazebo/set_model_state", timeout=30) #wait for service name
		#rospy.loginfo("Initialising counter behaviour.")

		# counter
		self.i = 0
		self.n = n
		#{ position: { x: -1.130530, y: -6.653650, z: 0.86250 }, orientation: {x: 0, y: 0, z: 0, w: 1 } }, twist: { linear: {x: 0 , y: 0, z: 0 } , angular: { x: 0, y: 0, z: 0 } } , reference_frame: map } }
		state_msg = ModelState()
		state_msg.model_name = "aruco_cube"
		state_msg.pose.position.x = -1.130530
		state_msg.pose.position.y = -6.65365
		state_msg.pose.position.z = 0.86250
		state_msg.pose.orientation.x = 0
		state_msg.pose.orientation.y = 0
		state_msg.pose.orientation.z = 0
		state_msg.twist.linear.x = 0
		state_msg.twist.linear.y = 0
		state_msg.twist.linear.z = 0
		state_msg.twist.angular.x = 0
		state_msg.twist.angular.y = 0
		state_msg.twist.angular.z = 0
		state_msg.reference_frame = "map"

		self.state_msg = state_msg
		
		
		# become a behaviour
		super(ResetCounter, self).__init__(name)

	def update(self):
		global failedPlace
		# increment i
		self.i += 1

		# succeed after count is done
		if self.i <= self.n:
			return pt.common.Status.FAILURE 
		else: 
			failedPlace =False
			self.i = 0
			rospy.loginfo("Tree resetter done")
			self.cube_reset_srv(self.state_msg)
			return pt.common.Status.SUCCESS
		


if __name__ == "__main__":


	rospy.init_node('main_state_machine')
	try:
		BehaviourTree()
	except rospy.ROSInterruptException:
		pass

	rospy.spin()
