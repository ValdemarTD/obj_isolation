#!/usr/bin/env python
from os import path
import subprocess
import xml.etree.ElementTree as ET

class ObjectTracker:
    def __init__(self, tree_file = "obj_tree.xml", obj_tolerance=0.25, clear_tree=False, debug=False):
        self.debug = debug
        if path.exists(tree_file):
            if clear_tree:
                cmd = "mv " + str(tree_file) + " " + str(tree_file) + ".old"
                if self.debug:
                    print "Clearing tree. Command string is: " + cmd + "\n\n"
                subprocess.call(cmd)
                elem = ET.Element('top')
                self.xml_tree = ET.ElementTree(element=elem)
            else:
                self.xml_tree = ET.ElementTree.parse(tree_file)
        else:
            elem = ET.Element('top')
            self.xml_tree = ET.ElementTree(element=elem)

        self.obj_tolerance = obj_tolerance
        self.filename = tree_file
        self.centroid_array = []


    #Takes in an array of object centroids and creates and XML tree
    #for use in object handling and storage
    def update_xml_centers(self):
        for obj_id in range(len(self.centroid_array)):
            exists = False
            for object in self.xml_tree.findall('obj'):
                if abs(float(object.get('x')) - float(self.centroid_array[obj_id][0])) < self.obj_tolerance and abs(float(object.get('y')) - float(self.centroid_array[obj_id][1])) < self.obj_tolerance:
                    exists = True
                    break
            if not exists:
                new_obj = ET.SubElement(self.xml_tree.getroot(), 'obj')
                new_obj.attrib["x"] = str(self.centroid_array[obj_id][0])
                new_obj.attrib["y"] = str(self.centroid_array[obj_id][1])


    #Function to take in a centroid array
    def set_centroid_array(self, arr):
        self.centroid_array = arr

    #Function to export a new version of the tree
    def export_obj_list(self, file=""):
        if file == "":
            file = self.filename
        if path.exists(file):
            cmd = "mv " + str(file) + " " + str(file) + ".old"
            subprocess.call(cmd, shell=True)
        self.xml_tree.write(file)

    #Function to return the full XML tree
    def get_full_tree(self):
        return self.xml_tree

    #Function to return a subset of elements from our tree
    def get_tree_elements(self, elem_name):
        return self.xml_tree.findall(elem_name)
