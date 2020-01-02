#!/usr/bin/env python
import rospy as rp
from sensor_msgs import Image
from std_msgs import UInt32, UInt8, Header
import cv2
from shapely.geometry import *
import xml.etree.ElementTree as ET


class DrawBoxes:
    def __init__(self):
        self.xml_tree = ET.Element('top')
        self.centroid_array = []
        self.processed_image_pub = rp.Publisher("processed_image", Image, queue_size=3)
        self.update_image = False
        self.bridge = CVBridge()

    #Takes in an array of object centroids and creates and XML tree
    #for use in object handling and storage
    def update_xml_centers(self):
        for obj_id in range(len(self.centroid_array)):
            exists = False
            for object in self.xml_tree.findall('obj'):
                if abs(float(object.get('x')) - float(self.centroid_array[obj_id][0])) < 0.1 and abs(float(object.get('y')) - float(self.centroid_array[obj_id][1])) < .1:
                    exists = True
                    break
            if not exists:
                new_obj = self.xml_tree.SubElement('obj')
                new_obj.attrib["x"] = str(self.centroid_array[obj_id][0])
                new_obj.attrib["y"] = str(self.centroid_array[obj_id][1])

    #Function to load an XML tree instead of creating one from centroids
    def xml_tree_from_file(self, file_path):
        self.xml_tree = ET.parse(file_path)

    #Function to take in an image and return an array of objects identified in
    #said image. Uses OpenCV and YOLOv3
    def id_object(self, image_in):
        cv2_image = self.bridge.imgmsg_to_cv2(image_in, "bgr8") #Going off of tutorial
                                                                #at http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
                                                                #for bgr8
        processed_image = self.yolo3_process(cv2_image)
        #TODO: -Postprocessing of image (remove redundant boxes, etc.)
        #
        #      -Verification that any remaining boxes are associated with the object
        #      which should be near the vertical center of the image (shapely to check)
        #
        #      -Returning fully processed image and associated box information

    #Function to run raw image through YOLOv3-based neural network object
    #identification process. 100% TODO
    def yolo3_process(self, image_in):


    #Callback for head camera status. Intent is to only save a single picture
    #when associating an image with a specific object. Purpose of saving
    #single picture is to allow up to not constantly update what type we think
    #the object is. Can be easily changed.
    def status_update(self, data):
        self.update_image = True
