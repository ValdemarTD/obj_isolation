#!/usr/bin/env python
from shapely.geometry import *

#Class definition for a class that processes a previously filtered pointcloud
#to isolate individual objects.
class PointcloudProcessor:
    def __init__(self):
        self.obj_array = []
        self.point_array = []
        self.centroid_array = []
        self.max_separation = 0.15

    #Get and set functions for our object array which should be filled with
    #Shapely Geometry Polygon objects
    def set_obj_array(self, new_arr):
        self.obj_array = new_arr
    def get_obj_array(self):
        return self.obj_array

    #Get function for our centroid array which should be full of XY coordinates
    #with the same indexes as the objects they're from
    def get_centroid_array(self):
        return self.centroid_array

    #Function to set our maximum separation
    def set_max_separation(self, new_sep):
        self.max_separation = new_sep

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
            #Checks if we're looking at the first point in the array, which
            #should always be added
            if i = 0:
                new_obj_points.append(self.point_array[i])

            #Checks if the next point in within our pre-set maximum
            #allowed separation
            if dist_okay(self.point_array[i], self.point_array[i + 1]):
                new_obj_points.append(self.point_array[i + 1])

            #If the next point is not within our max distance, we consider it to
            #be part of a different object. We take our current array of connected
            #points and make a polygon out of them, then clear that array and
            #start a new one with that next point
            else:
                self.obj_array.append(Polygon(new_obj_points))
                new_obj_points = []
                new_obj_points.append(self.point_array[i + 1])

        #Appends the final object to the array
        self.obj_array.append(Polygon(new_obj_points))
        self.update_centroid_array()

    #Updates our array of object centroids
    def update_centroid_array(self):
        self.centroid_array = []
        for poly in self.obj_array:
            self.centroid_array.append([poly.centroid.x, poly.centroid.y])
