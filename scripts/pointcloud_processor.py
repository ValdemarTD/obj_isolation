#!/usr/bin/env python
from shapely.geometry import *

#Class definition for a class that processes a previously filtered pointcloud
#to isolate individual objects.

class PointcloudProcessor:
    def __init__(self):
        self.obj_array = []
        self.point_array = []
        self.max_separation = 0.10

    #Get and set functions for our object array which should be filled with
    #Shapely Geometry Polygon objects
    def set_obj_array(self, new_arr):
        self.obj_array = new_arr
    def get_obj_array(self):
        return self.obj_array

    #Takes in an array of XY coordinates (or potentially XYZ, which are interpreted
    #in the same way by Shapely) and places them into a new array for processing
    def fill_points_array(self, new_arr):
        self.point_array = []
        for new_point in new_arr:
            self.point_array.append([new_point[0], new_point[1]])


    #Checks if the distance between two points is within an acceptable
    #separation to be considered part of the same object
    def dist_okay(self, point_1, point_2):
        if Point(point_1).distance(Point(point_2)) > self.max_separation:
            return False
        else:
            return True

    #Isolates each object based on the distance between points in our point cloud
    def isolate_objects(self):
        self.obj_array = []
        new_obj_points = []
        for i in range(len(self.point_array) - 1):
            if i = 0:
                new_obj_points.append(self.point_array[i])
            if dist_okay(self.point_array[i], self.point_array[i + 1]):
                new_obj_points.append(self.point_array[i + 1])
            else:
                self.obj_array.append(Polygon(new_obj_points))
                new_obj_points = []
                new_obj_points.append(self.point_array[i + 1])
        if new_obj_points[0] == self.point_array[-1]:
            self.obj_array.append(Polygon(new_obj_points))
