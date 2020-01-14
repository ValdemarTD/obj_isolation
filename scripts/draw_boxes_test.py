#!/usr/bin/env python
import rospy as rp
from sensor_msgs.msg import Image
from std_msgs.msg import UInt32, UInt8, Header
from cv_bridge import CvBridge
import cv2 as cv
from draw_boxes import DrawBoxes

class_file = "../darknet_config/classifications/coco.names"
weights_file = "../darknet_config/weights/yolov3.weights"
net_config_file = "../darknet_config/cfg/yolov3.cfg"

bridge = CvBridge()
drawer = DrawBoxes(class_file, weights_file, net_config_file, debug_mode=True)

image_in_name = "test_in.jpg"
image_out_name = "test_out.jpg"

image_in =  cv.imread(image_in_name)
image_to_draw = bridge.cv2_to_imgmsg(image_in)
image_out = drawer.id_object(image_to_draw)
#drawer.show_image()
drawer.save_image(image_out_name)
