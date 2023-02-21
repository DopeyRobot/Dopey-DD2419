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
        self.f = 10
        self.rate = rospy.Rate(self.f)
        self.publisher_vertices = rospy.Publisher("workspace", PolygonStamped, queue_size=10)
        self.frame_id = "map"
        self.vertices_df = pd.read_csv("example_workspace.tsv", sep="\t")
        self.vertices = self.vertices_df.values
        self.vertices_list = np.append(self.vertices, [self.vertices[0]], axis = 0)

        self.p = [-1.4, 2] ##REPLACE WITH CURRENT LOCATION OF THE ROBOT
        self.pinf = [10000, self.p[1]]
        self.run()

    def direction(self, A, B, C):
        dir = (C[1] - A[1])*(B[0] - A[0]) - (B[1]- A[1])* (C[0]- A[0])
        if dir == 0:
            #collinear 
            return 0
        elif dir > 0:
            #counterclockwise 
            return 1
        else:
            return 2
    
    def checkonline(self, p1, p2, p3):
        #check if p1, lies on line segment with points p2, p3

        if (p1[0] <= max(p2[0], p3[0]) and p1[0] <= min(p2[0], p3[0]) and p1[1] <= max(p2[1], p3[1]) and p1[1] <= min(p2[1], p3[1])):
            return True        
        
        # if (p1[0]== p2[0] or p1[0] == p3[0]) and (p1[1] == p2[1] or p1[1] == p3[1]):
        #     return True
        else:
            return False

    
    def checkintersection(self, A, B, C, D):
        #ABCD points [x,y]
        #AB on line 1, CD on line 2
        # True if intersect
        d1 = self.direction(A, B, C)
        d2 = self.direction(A, B, D)
        d3 = self.direction(C, D, A)
        d4 = self.direction(C, D, B)

        if d1 != d2 and d3 != d4:
            # different orientations imply intersection
            return True
        if d1 == 0 and self.checkonline(C, A, B):
            #if collinear, check if C is on AB
            return True
        if d2 == 0 and self.checkonline(D, A, B):
            return True
        if d3 == 0 and self.checkonline(A, C, D):
            return True
        if d4 == 0 and self.checkonline(B, C, D):
            return True

        return False
    
    def checkinsidepoly(self):
        n_edges = len(self.vertices)
        print(self.vertices)
        print(n_edges)
        if n_edges < 3:
            return False
        i = 0
        count = 0
        while True:
            #find point of edge of polygon
            edge1 = self.vertices[i]
            edge2 = self.vertices[i + 1]
            print(edge2)
            if self.checkintersection(self.p, self.pinf, edge1, edge2):
                #intersection exists
                if self.direction(edge1, self.p, edge2) == 0:
                    return self.checkonline(self.p, edge1, edge2)
                count += 1
            print("before mod i:", i)
            print("(i+1) % n_edges:", (i+1) % n_edges)
            i = (i+2) % n_edges
            print("after mod i:", i)
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
