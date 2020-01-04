#!/usr/bin/env python
import xml.etree.ElementTree as ET

class ObjectTracker:
    def __init__(self, tree_file = ""):
        if not tree_file == "":
            self.xml_tree = ET.parse(tree_file)
        else:
            self.xml_tree = ET.Element('top')

        self.centroid_array = []


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


    #Function to take in a centroid array
    def set_centroid_array(self, arr):
        self.centroid_array = arr
