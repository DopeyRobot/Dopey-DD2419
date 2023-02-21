#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PolygonStamped, Point32
from robp_msgs.msg import DutyCycles,Encoders
import tf2_ros
import math
import pandas as pd
import numpy as np
from shapely.geometry import Polygon
import matplotlib.pyplot as plt

class Workspace():
    def __init__(self):
        self.verbose = 0
        self.f = 10
        self.rate = rospy.Rate(self.f)
        self.publisher_vertices = rospy.Publisher("workspace", PolygonStamped, queue_size=10)
        self.frame_id = "map"
        self.vertices_df = pd.read_csv("example_workspace.tsv", sep="\t")
        self.vertices = self.vertices_df.values
        self.vertices_list = np.append(self.vertices, [self.vertices[0]], axis = 0)

        self.p = [500,500] ##REPLACE WITH CURRENT LOCATION OF THE ROBOT
        self.pinf = [10000, self.p[1]]
        self.run()

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
    
    def checkinsidepoly(self):
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
        
        if (self.checkinsidepoly()):
            print("inside polygon")
            ## insert code for stopping motors/dutycycle
        else:
            print("outside polygon")
        

        
        return point_list
            

        # print(point_list)

            

    def run(self):
        while not rospy.is_shutdown():
            poly_points = PolygonStamped()
            point_list = self.polygon()
            poly_points.polygon.points = point_list
            poly_points.header.frame_id = self.frame_id
            #print(poly_points)
            self.publisher_vertices.publish(poly_points)
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
