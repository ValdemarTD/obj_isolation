#!/usr/bin/env python
import rospy as rp
from object_tracker import ObjectTracker
from pointcloud_processor import PointcloudProcessor
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32

#Thought process: Take in filtered pointcloud and run constant updates for a robot running through
#a known map with a small set of objects (but slowly enough to avoid screwing up localization)
#Then export the object list and check how many elements have been added to the tree. Restart
#same node with existing dataset to check long-term storage method and see if robot avoids adding additional
#objects. Should also publish list of centroids (Potentially in pointcloud form.)

debug = True

rp.init_node("object_tracker_test", anonymous=True)

sub_topic = "filtered_pointcloud"

obj_tracker = ObjectTracker(clear_tree=True, debug=debug)
cloud_proc = PointcloudProcessor()

#Callback for filtered pointcloud topic.
def pointcloud_callback(data):
    if len(data.points) == 0:
        return
    new_points = []
    for point in data.points:
        new_points.append([point.x, point.y, point.z])
    cloud_proc.fill_points_array(new_points)
    cloud_proc.isolate_objects()
    obj_tracker.set_centroid_array(cloud_proc.get_centroid_array())
    obj_tracker.update_xml_centers()


filtered_cloud_sub = rp.Subscriber(sub_topic, PointCloud, pointcloud_callback, queue_size=4)


while not rp.is_shutdown():
    rp.spin()

obj_tracker.export_obj_list()
