#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PolygonStamped, Point32, PoseStamped
from nav_msgs.msg import Odometry
from robp_msgs.msg import DutyCycles
import tf2_ros
import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

class Workspace():
    def __init__(self):
        self.verbose = 1
        self.f = 10
        self.rate = rospy.Rate(self.f)
        self.navgoal = PoseStamped() #does that work?
        self.publisher_dutycycle = rospy.Publisher("/motor/duty_cycles", DutyCycles, queue_size=10)
        self.publisher_vertices = rospy.Publisher("workspace", PolygonStamped, queue_size=10)
        self.subscriber_robopos = rospy.Subscriber("/odometry", Odometry, self.odom_callback)
        self.frame_id = "map"
        # self.vertices_df = pd.read_csv("~/dd2419_ws/src/workspace/example_workspace.tsv", sep="\t")
        self.vertices_df = pd.read_csv("example_workspace.tsv", sep="\t")

        self.vertices = self.vertices_df.values
        self.vertices_list = np.append(self.vertices, [self.vertices[0]], axis = 0)
        self.dutyoff = False
        self.subscriber_navgoal = rospy.Subscriber("move_base_simple/goal", PoseStamped, self.navgoal_callback)
        self.publisher_goal = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        self.robo_posestamped = Odometry()
        self.p = [2, 0] ##REPLACE WITH CURRENT LOCATION OF THE ROBOT
        self.pinf = [10000, self.p[1]]
        
        self.navgoalnew = False
        self.run()
    
    def navgoal_callback(self, navgoalmsg):
        self.navgoal = PoseStamped()
        self.navgoal.pose = navgoalmsg.pose.pose
        self.navgoal.header = navgoalmsg.header
        self.navgoal.header.frame_id = "odom"
    
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
    

        #subscribe to navgoal message & set boolean true if its outside, then overwrite new navgoal
        navgoal_point = [self.navgoal.pose.position.x, self.navgoal.pose.position.y]
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
            print(f"Current position of navgoal:{navgoal_point}")
        while True:
            #find point of edge of polygon
            edge1 = self.vertices[i]
            edge2 = self.vertices[i + 1]
            if self.verbose:
                print(f"edge{i}:", edge1)
                print(f"edge{i+1}:", edge2)
            pinf = [10000, ]
            if self.checkintersection(edge1, edge2, navgoal_point, self.pinf):
                #if the point is on the edge then its definitely inside aka return True
                if self.direction(edge1, navgoal_point, edge2) == 0:
                    return self.checkonline(navgoal_point, edge1, edge2)
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
    
    def odom_callback(self, msg):
        self.robo_posestamped = msg
        self.p[0] = msg.pose.pose.position.x
        self.p[1] = msg.pose.pose.position.y

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
    
    # def checkroboinsidepoly(self):
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
            print(f"Current position of robot:{self.p}")
        while True:
            #find point of edge of polygon
            edge1 = self.vertices[i]
            edge2 = self.vertices[i + 1]
            if self.verbose:
                print(f"edge{i}:", edge1)
                print(f"edge{i+1}:", edge2)
                
            if self.checkintersection(edge1, edge2, self.p, self.pinf):
                #if the point is on the edge then its definitely inside aka return True
                if self.direction(edge1, self.p, edge2) == 0:
                    return self.checkonline(self.p, edge1, edge2)
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
        # print((f"vertices_list:{vertices_list}"))
        # print(vertices_list)
        xs, ys = zip(*self.vertices_list)
        # print(xs, ys)

        point_list = []
        for vertice in self.vertices_list:
            point_type = Point32()
            point_type.x = vertice[0]
            point_type.y = vertice[1]
            point_list.append(point_type)
        navgoal_pos = [self.navgoal.pose.position.x, self.navgoal.pose.position.y]
        if (self.checkpointinsidepoly(navgoal_pos, [1000, navgoal_pos[1]])):
            # navgoal inside polygon
            if (self.checkpointinsidepoly(self.p, [1000, self.p[1]])):
                self.dutyoff = False
                if self.verbose:
                    print("robot inside workspace")
                ## insert code for stopping motors/dutycycle
            else:
                if self.verbose:
                    print("robot outside polygon")
                rospy.loginfo("Warning: robot outside workspace")
                self.dutyoff = True
        else:
            #navgoal outside poly, publish new navgoal
            self.navgoalnew = True
            if self.verbose:
                print("navgoal outside workspace")

        
        return point_list
            

        # print(point_list)


    def run(self):
        while not rospy.is_shutdown():
            # Pose Stamped Publisher
            poly_points = PolygonStamped()
            point_list = self.polygon()
            poly_points.polygon.points = point_list
            poly_points.header.frame_id = self.frame_id
            #print(poly_points)
            self.publisher_vertices.publish(poly_points)

            # Nav Goal Publisher
            posestamped_message = PoseStamped()
            if self.navgoalnew or self.dutyoff:
                posestamped_message.pose = self.robo_posestamped.pose.pose
                posestamped_message.header = self.robo_posestamped.header
                posestamped_message.header.frame_id = "odom"
                self.publisher_goal.publish(posestamped_message)
                rospy.loginfo("new goal to avoid out")
                # publish zero duty cycle 
            
            #Publish new navgoal 
            self.rate.sleep()
    
        

        # x = [0.0, 0.0, 1.0, 1.0, 0.0]
        # y = [0.0, 1.0, 1.0, 0.0, 0.0]

        # poly = Polygon(zip(x,y))

        # # Extract the point values that define the perimeter of the polygon
        # xx, yy = poly.exterior.coords.xy
        
        # poly_message = Polygon()
        # poly_message.x = xs
        # poly_message.y = ys

        # print(xx, yy)
        # plt.figure()
        # plt.plot(xs, ys)
        # plt.show()


if __name__ == '__main__': 
    try:
        rospy.init_node("workspace")
        Workspace()
        # ws = Workspace() 
        # ws.polygon()
        rospy.spin()
    except rospy.ROSInternalException:
        pass

    ##CREATE MESSAGE TYPE
    ##PUBLISH ONTO ANY TOPIC & DISPLAY IN RVIZ
