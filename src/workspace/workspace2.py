#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped, Pose
from nav_msgs.msg import Odometry
from robp_msgs.msg import DutyCycles
import tf2_ros
import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

class Workspace():
    def __init__(self):
        self.verbose = 0
        self.f = 10
        self.rate = rospy.Rate(self.f)
        self.robot_position = None
        self.navgoal_position = None
        self.navgoal = PoseStamped() #does that work?
        self.publisher_dutycycle = rospy.Publisher("/motor/duty_cycles", DutyCycles, queue_size=10)
        self.publisher_vertices = rospy.Publisher("workspace", PolygonStamped, queue_size=10)
        self.subscriber_robopos = rospy.Subscriber("/odometry", Odometry, self.odom_callback)
        self.frame_id = "map"
        self.vertices_df = pd.read_csv("~/dd2419_ws/src/workspace/example_workspace.tsv", sep="\t")
        # self.vertices_df = pd.read_csv("example_workspace.tsv", sep="\t")

        self.vertices = self.vertices_df.values
        self.vertices_list = np.append(self.vertices, [self.vertices[0]], axis = 0)
        # self.dutyoff = False
        self.subscriber_navgoal = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.navgoal_callback)
        self.publisher_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.robo_posestamped = Odometry()


        self.robot_inside = True #will approach
        self.navgoal_inside = True #will aproach
        self.pinf = [10000, 10000] #point can be anywhere
        
        self.navgoalnew = False
        self.run()
    
    def navgoal_callback(self, navgoalmsg):
        rospy.loginfo("navgoal cb")
        self.navgoal = PoseStamped()
        self.navgoal.pose = navgoalmsg.pose
        self.navgoal.header = navgoalmsg.header
        self.navgoal.header.frame_id = "odom"
        self.navgoal_position = [self.navgoal.pose.position.x, self.navgoal.pose.position.y]

    def odom_callback(self, msg):
        self.robo_posestamped = msg
        self.robot_position = [msg.pose.pose.position.x, msg.pose.pose.position.y]
        
    def direction(self, A, B, C):
        # find the orientation of 3 points
        # 0 : collinear points
        # 1 : clockwise 
        # 2 : counterclockwise 
        dir = (B[1]- A[1])*(C[0] - B[0]) - (B[0]-A[0])*(C[1]-B[1])
        # dir = (C[1] - A[1])*(B[0] - A[0]) - (B[1]- A[1])* (C[0]- A[0])
        # if self.verbose: 
        #     print(f"Given Orientation value:{dir}")
        if dir == 0:
            #collinear 
            if self.verbose:
                print(f"{A}, {B}, {C}: collinear, given orientation value: {dir}")
            return 0
        elif dir > 0:
            #clockwise
            if self.verbose:
                print(f"{A}, {B}, {C}: clockwise, given orientation value: {dir}") 
            return 1
        else:
            if self.verbose:
                print(f"{A}, {B}, {C}: anticlockwise, given orientation value: {dir}")
            return 2
    
    def checkonline(self, p1, p2, p3):
        # given collinear points p1, p2, p3, check if p2, lies on line segment p1p3
        if ((p2[0] <= max(p1[0], p3[0])) and (p2[0] >= min(p1[0], p3[0])) and
            (p2[1] <= max(p1[1], p3[1])) and (p2[1] >= min(p1[1], p3[1]))):
            if self.verbose:
                print(f"given collinear points: p2={p2} lies on p1= {p1} and p3= {p3}")
            return True
        return False

    def checkintersection(self, A, B, C, D):
        if self.verbose:
            print("Intersection exists")
        #ABCD points [x,y]
        #AB on line 1, CD on line 2
        # True if intersect
        d1 = self.direction(A, B, C)
        d2 = self.direction(A, B, D)
        d3 = self.direction(C, D, A)
        d4 = self.direction(C, D, B)

        if d1 != d2 and d3 != d4:
            if self.verbose:
                print(f"Intersection Reason: Different orientations for {A}, {B}, {C} and {A}, {B}, {D} || {C}, {D}, {A} and {C}, {D}, {B}")
            # different orientations imply intersection
            return True
        if d1 == 0 and self.checkonline(A, B, C):
            # A, B, C collinear and B lies on segment AC
            if self.verbose:
                print(f"Intersection Reason: {A}, {B}, {C} collinear and {B} lies on {A}-{C} segement")
            return True
        if d2 == 0 and self.checkonline(A, B, D):
            if self.verbose:
                print(f"Intersection Reason: {A}, {B}, {D} collinear and {B} lies on {A}-{D} segement")
            return True
        if d3 == 0 and self.checkonline(C, D, A):
            if self.verbose:
                print(f"Intersection Reason: {C}, {D}, {A} collinear and {C} lies on {D}-{A} segement")
            return True
        if d4 == 0 and self.checkonline(C, D, B):
            if self.verbose:
                print(f"Intersection Reason: {C}, {D}, {B} collinear and {C} lies on {D}-{B} segement")
            return True
        # return False if none of the cases are true
        return False
         
    def checkpointinsidepoly(self, point_interest, point_infinity):
        # return 1 if inside polygon
        n_edges = len(self.vertices)
        if self.verbose:
            print("\n\n") 
            print(f"vertices: {self.vertices}")
            print(f"number of edges: {n_edges}")
        if n_edges < 3:
            return False
        i = 0
        count = 0
        if self.verbose:
            print(f"Current position of robot:{point_interest}")
        while True:
            #find point of edge of polygon
            edge1 = self.vertices[i]
            edge2 = self.vertices[i + 1]
            if self.verbose:
                print(f"edge{i}:", edge1)
                print(f"edge{i+1}:", edge2)
                
            if self.checkintersection(edge1, edge2, point_interest, point_infinity):
                #if the point is on the edge then its definitely inside aka return True
                if self.direction(edge1, point_interest, edge2) == 0:
                    return self.checkonline(point_interest, edge1, edge2)
                count += 1
                if self.verbose:
                    print("intersection count:", count)
            else:
                if self.verbose:
                    print("No intersection")

            i = (i+1) % (n_edges-1)
            if self.verbose:
                print("new i:", i)
            if i == 0:
                #break if it exceeds the edge count
                break
            # return 1 if odd number of intersection => inside
            # return 0 if even number of intersections => outside 
        return count & 1

    def checkpointinsidepoly2(self, point_interest, point_infinity):
        # return 1 if inside polygon
        n_edges = len(self.vertices)
        if self.verbose:
            print("\n\n") 
            print(f"vertices: {self.vertices}")
            print(f"number of edges: {n_edges}")
        if n_edges < 3:
            return False
        i = 0

        count = 0
        if self.verbose:
            print(f"Current position of robot:{point_interest}")
        while True:
            #find point of edge of polygon
            edge1 = self.vertices[i]
            edge2 = self.vertices[i + 1]
            if self.verbose:
                print(f"edge{i}:", edge1)
                print(f"edge{i+1}:", edge2)
                
            if self.checkintersection(edge1, edge2, point_interest, point_infinity):
                #if the point is on the edge then its definitely inside aka return True
                if self.direction(edge1, point_interest, edge2) == 0:
                    return self.checkonline(point_interest, edge1, edge2)
                count += 1
                if self.verbose:
                    print("intersection count:", count)
            else:
                if self.verbose:
                    print("No intersection")

            i = (i+1) % (n_edges-1)
            if self.verbose:
                print("new i:", i)
            if i == 0:
                #break if it exceeds the edge count
                break
            # return 1 if odd number of intersection => inside
            # return 0 if even number of intersections => outside 
        return count & 1

    def polygon(self):
        point_list = []
        xs, ys = zip(*self.vertices_list)
        for vertice in self.vertices_list:
            point_type = Point32()
            point_type.x = vertice[0]
            point_type.y = vertice[1]
            point_list.append(point_type)
        return point_list
    
    # def polygoncentroid(self):
    #     #centroid of non self intersecting polygon
    #     for vertice in self.vertices_list:
        
    #     return xc, yc



        # if not (self.navgoal_pos is None or self.p is None): # fix since it should be done independently
        #     if (self.checkpointinsidepoly(self.navgoal_pos, [1000, self.navgoal_pos[1]])):
        #         # navgoal inside polygon
        #         if (self.checkpointinsidepoly(self.p, [1000, self.p[1]])):
        #             self.dutyoff = False
        #             if self.verbose:
        #                 print("robot inside workspace")
        #             ## insert code for stopping motors/dutycycle
        #         else:
        #             if self.verbose:
        #                 print("robot outside polygon")
        #             rospy.loginfo("Warning: robot outside workspace")
        #             self.dutyoff = True
        #     else:
        #         #navgoal outside poly, publish new navgoal
        #         self.navgoalnew = True
        #         if self.verbose:
        #             print("navgoal outside workspace")

        
        # return point_list
            

        # print(point_list)


    def run(self):
        while not rospy.is_shutdown():
            # Pose Stamped Publisher
            poly_points = PolygonStamped()
            point_list = self.polygon()
            poly_points.polygon.points = point_list
            poly_points.header.frame_id = self.frame_id
            self.publisher_vertices.publish(poly_points)

            # Calculate the center point of the polygon


            ## CHECK
            if self.verbose:
                print(f"robot position:{self.robot_position} navgoal position:{self.navgoal_position}")
            if self.robot_position is not None  and self.navgoal_position is not None:
                self.robot_inside = self.checkpointinsidepoly(self.robot_position, self.pinf) #check robot position before boolean
                if self.robot_inside:
                    if self.verbose:
                        print("Robot Inside True")
                    #Robot inside poly
                    self.navgoal_inside = self.checkpointinsidepoly(self.navgoal_position, self.pinf) #check navgoal before boolean 
                    if not self.navgoal_inside:
                        #Navgoal outside poly
                        ########### METHOD ONE:: PUBLISH POSITION OF ROBOT AS NEW NAVGOAL ############################
                        posestamped_message = PoseStamped()
                        posestamped_message.pose = self.robo_posestamped.pose.pose
                        posestamped_message.header = self.robo_posestamped.header
                        posestamped_message.header.frame_id = "odom"
                        self.publisher_goal.publish(posestamped_message)
                        rospy.loginfo("Navgoal outside workspace: New Navgoal generated.")

                        ########### METHOD TWO:: PUBLISH POSITION OF ROBOT AS NEW NAVGOAL ############################
            self.rate.sleep()


if __name__ == '__main__': 
    try:
        rospy.init_node("workspace")
        Workspace()
        rospy.spin()
    except rospy.ROSInternalException:
        pass

