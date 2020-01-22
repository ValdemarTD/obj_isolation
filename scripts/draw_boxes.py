#!/usr/bin/env python
import rospy as rp
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32, UInt8, Header
from cv_bridge import CvBridge
import numpy as np
import cv2 as cv
from shapely.geometry import *

class DrawBoxes:
    def __init__(self, class_file, weights_file, net_config_file, target=cv.dnn.DNN_TARGET_CPU, backend=cv.dnn.DNN_BACKEND_OPENCV, debug_mode=False, thresh=0.5, nms_thresh=0.4):
#        self.processed_image_pub = rp.Publisher("processed_image", Image, queue_size=10)
#        self.update_image = False
        #Bool to set whether debug outputs are on
        self.debug_mode = debug_mode
        self.cv2_image = None


        self.bridge = CvBridge()
        self.last_image = None


        #Initialize variables for Darknet processing
        self.thresh = thresh
        self.nms_thresh = nms_thresh

        

        #Initializes classification list
        self.classes = None
        with open(class_file, 'rt') as f:
            self.classes = f.read().rstrip('\n').split('\n')

        #Initialize our net from a given weights file and configuration file
        #and set up our target and backend (Default to CPU and OpenCV respective)
        self.net = cv.dnn.readNetFromDarknet(net_config_file, weights_file)
        self.net.setPreferableBackend(backend)
        self.net.setPreferableTarget(target)


    #Function to take in an image and return an array of objects identified in
    #said image. Uses OpenCV and Darknet
    def id_object(self, image_in, center_check=False):
        #Need both height and width to be multiples of 32, then crop to square
        w = int(image_in.width) - int(image_in.width) % 32
        h = int(image_in.height) - int(image_in.height) % 32
        side = min([w,h])
        self.cv2_image = self.bridge.imgmsg_to_cv2(image_in, image_in.encoding) #Going off of tutorial
                                                                                #at http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
        self.cv2_image = self.cv2_image[(int(image_in.width)-w)/2:int(image_in.width) - (int(image_in.width)-w)/2, (int(image_in.height)-h)/2:int(image_in.height) - (int(image_in.height)-h)/2]
        outs, new_w, new_h = self.darknet_process(self.cv2_image, w, h)
        boxes = self.postprocess(outs, center_check, new_w, new_h)
#        self.last_image = cv.UMat(np.array(outs), dtype=np.float32)
        self.last_image = outs[0]
        return outs, boxes

    #Function to run raw image through Darknet neural network object. Intend to default to using YOLOv3 with COCO classes
    #Adapting tutorial at https://www.learnopencv.com/deep-learning-based-object-detection-using-yolov3-with-opencv-python-c/
    #for use with single images instead of video streams. Separating postprocessing into self.id_object function
    def darknet_process(self, image_in, w, h):
        #Convert image to blob format and set as input for our net
        blob = cv.dnn.blobFromImage(image_in, 1, (w, h), [0,0,0], 1, crop=False)
        blobb = blob.reshape(blob.shape[2] * blob.shape[1] , blob.shape[3], 1)

        if self.debug_mode:
#            print "Original image displaying\n\n"
#            cv.imshow('Original', image_in)
#            cv.waitKey(15000)
#            print "Key pressed. Continuing.\n\n"
#            cv.destroyAllWindows()
            print "Length of blob given to Darknet: " + str(len(blob)) + "\n\n"
            print "Blob shape: " + str(blob.shape) + "\n\n"
            print "Blob given to Darknet: " + str(blob) + "\n\n"
#            print "Displaying blob then waiting...\n\n"
#            blobb = blob.reshape(blob.shape[2] * blob.shape[1] , blob.shape[3], 1)
            print "Blob reshaped to:" + str(blobb.shape) + "\n\n"
#            cv.imshow('Blob',blobb)
#            cv.waitKey(10000)
#            print "Key pressed. Continuing.\n\n"
#            cv.destroyAllWindows()

        self.net.setInput(blob)

        names = self.get_outputs_names()

        if self.debug_mode:
            print "Names given to Darknet:\n" + str(names) + "\n\n"

        out = self.net.forward(names)

        if self.debug_mode:
            print "Darknet output:\n" + str(out) + "\n\n"
            print "Type of darknet output: " + str(type(out)) + "\n\n"
            print "Type of darknet list items: " + str(type(out[0])) + "\n\n"
            print "Type of numpy array items in darknet list: " + str(np.array(out).dtype) + "\n\n"
            print "Length of darknet items: "
            for item in out:
                print str(len(item)) + " "
                #for i in item:
                #    print str(len(i)) + " "
                #print "\n"
            print "\n\n"

        return out, blobb.shape[0], blobb.shape[1]


    #Get the names of the output layers
    #Modified from tutorial at https://www.learnopencv.com/deep-learning-based-object-detection-using-yolov3-with-opencv-python-c/
    def get_outputs_names(self):
        #Get the names of all the layers in the network
        layersNames = self.net.getLayerNames()

        if self.debug_mode:
            print "Length of Layer Names:" + str(len(layersNames)) + "\n"
            print "Layer Names:\n"
            print layersNames
            print "\nEnd of Layer Names\n\n"

        #Get the names of the output layers, i.e. the layers with unconnected outputs
        return [layersNames[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

    #Remove the bounding boxes with low confidence using non-maxima suppression
    #Modified from tutorial at https://www.learnopencv.com/deep-learning-based-object-detection-using-yolov3-with-opencv-python-c/
    def postprocess(self, outs, center_check, w, h):

        classIds = []
        confidences = []
        boxes = []
        #Scan through all the bounding boxes output from the network and keep only the
        #ones with high confidence scores. Assign the box's class label as the class with the highest score.
        classIds = []
        confidences = []
        boxes = []
        for out in outs:
            if self.debug_mode:
                print "Out given to postprocess:\n"
                print out
            for detection in out:
                scores = detection[5:]
                classId = np.argmax(scores)
                confidence = scores[classId]
                if confidence > self.thresh:
                    center_x = int(detection[0] * w)
                    center_y = int(detection[1] * h)
                    width = int(detection[2] * w)
                    height = int(detection[3] * h)
                    left = int(center_x - width / 2)
                    top = int(center_y - height / 2)
                    classIds.append(classId)
                    confidences.append(float(confidence))
                    if center_check:
                        #Matches a shapely geometry box with the given detection box
                        bounding_box = box(float(left), float(top - height), float(left + width), float(top))
                        #Makes a shapely geometry box to represent the center of the image.
                        centerline = LineString([(float(w/2), 0), (float(w/2), float(h))])

                        #If we're using center_check, we should be pointing directly at the object.
                        #If the box isn't representing that center object, then we don't care about
                        #it and should discard it. Essentially, we should be trying to identify an object
                        #which has some part of itself at the dead center of the image, so
                        #we only care about boxes that cross the center of the image.
                        #If it doesn't cross the center, just don't bother appending it.
                        if not bounding_box.intersects(centerline):
                            continue

                    boxes.append([left, top, width, height])
        if self.debug_mode:
            print "\nEnd of outs given to postprocess\n\n"


        #Perform non maximum suppression to eliminate redundant overlapping boxes with
        #lower confidences.
        indices = cv.dnn.NMSBoxes(boxes, confidences, self.thresh, self.nms_thresh)
        for i in indices:
            i = i[0]
            box = boxes[i]
            left = box[0]
            top = box[1]
            width = box[2]
            height = box[3]
            self.draw_pred(classIds[i], confidences[i], left, top, left + width, top + height)

        return boxes

    #Draw predicted bounding box
    #Modified from tutorial at https://www.learnopencv.com/deep-learning-based-object-detection-using-yolov3-with-opencv-python-c/
    def draw_pred(self, classId, conf, left, top, right, bottom):
        #Draw a bounding box.
        cv.rectangle(self.cv2_image, (left, top), (right, bottom), (0, 0, 255))

        #Make label for confidence
        label = '%.2f' % conf

        #Get the label for the class name and its confidence
        if self.classes:
            assert(classId < len(self.classes))
            label = '%s:%s' % (self.classes[classId], label)

        #Display the label at the top of the bounding box
        labelSize, baseLine = cv.getTextSize(label, cv.FONT_HERSHEY_SIMPLEX, 0.5, 1)
        top = max(top, labelSize[1])
        cv.putText(self.cv2_image, label, (left, top), cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255))

    #Simple function to make a window with the last image processed
    def show_image(self):
        cv.namedWindow('image', cv.WINDOW_AUTOSIZE)
        cv.imshow('image', self.last_image)
        cv.waitKey()

    #Simple function to save the last processed image to a file
    def save_image(self, filename):

        if self.debug_mode:
            print "Type of last image: " + str(type(self.last_image)) + "\n\n"

        cv.imwrite(filename, self.last_image)
