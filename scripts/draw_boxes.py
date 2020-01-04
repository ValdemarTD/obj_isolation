#!/usr/bin/env python
import rospy as rp
from sensor_msgs import Image
from std_msgs import UInt32, UInt8, Header
import cv2
from shapely.geometry import *

class DrawBoxes:
    def __init__(self, class_file, weights_file, net_config_file):
        self.processed_image_pub = rp.Publisher("processed_image", Image, queue_size=3)
        self.update_image = False
        self.bridge = CVBridge()

        #Initialize variables for YOLO processing
        self.thresh = 0.5
        self.nms_thresh = 0.4

        #Initializes classification list
        self.classes = None
        with open(class_file, 'rt') as f:
            self.classes = f.read().rstrip('\n').split('\n')

        #Initialize our net from a given weights file and configuration file
        #Currently set up to output to run with OpenCV as a backend and
        #run with CPU instead of GPU.
        #TODO: Make backend and target configurable
        self.net = cv2.dnn.readNetFromDarknet(net_config_file, weights_file)
        self.net.setPreferableBackend(cv.dnn.DNN_BACKEND_OPENCV)
        self.net.setPreferableTarget(cv.dnn.DNN_TARGET_CPU)


    #Function to take in an image and return an array of objects identified in
    #said image. Uses OpenCV and YOLOv3
    def id_object(self, image_in, center_check=False):
        w = int(image_in.width)
        h = int(image_in.height)
        cv2_image = self.bridge.imgmsg_to_cv2(image_in, image_in.encoding) #Going off of tutorial
                                                                           #at http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
        processed_image = self.yolo3_process(cv2_image, h, w)
        postprocessed_image, boxes = self.postprocess(processed_image, center_check, w, h)
        return postprocessed_image, boxes
        #TODO: -Postprocessing of image (remove redundant boxes, etc.)
        #
        #      -Verification that any remaining boxes are associated with the object
        #      which should be near the vertical center of the image (shapely to check)
        #
        #      -Returning fully processed image and associated box information

    #Function to run raw image through YOLOv3-based neural network object
    #identification process. 100% TODO
    def yolo3_process(self, image_in):
        blob = cv.dnn.blobFromImage(image_in, 1/255, (w, h), [0,0,0], 1, crop=False)
