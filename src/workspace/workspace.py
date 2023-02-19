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
        self.run()

    def polygon(self):


        vertices_df = pd.read_csv("example_workspace.tsv", sep="\t")
        vertices_list = vertices_df.values
        print(type(vertices_list))
        vertices_list = np.append(vertices_list, [vertices_list[0]], axis = 0)
        print(vertices_list)
        xs, ys = zip(*vertices_list)

        point_list = []
        for vertice in vertices_list:
            point_type = Point32()
            point_type.x = vertice[0]
            point_type.y = vertice[1]
            point_list.append(point_type)
        
        return point_list
            

        # print(point_list)

            

    def run(self):
        while not rospy.is_shutdown():
            poly_points = PolygonStamped()
            point_list = self.polygon()
            poly_points.polygon.points = point_list
            poly_points.header.frame_id = self.frame_id
            print(poly_points)
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
