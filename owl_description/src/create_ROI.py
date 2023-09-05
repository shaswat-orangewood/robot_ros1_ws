#! /usr/bin/env python3

import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import numpy as np 
import yaml
from yaml.loader import BaseLoader

bounding_box_min = None
bounding_box_max = None

def point_cloud_callback(msg):
    global bounding_box_max, bounding_box_min
    # Convert PointCloud2 message to a numpy array
    pc_data = pc2.read_points(msg, skip_nans=True)
    point_cloud = np.array(list(pc_data))
    if len(point_cloud) == 0:
        return
    # Calculate minimum and maximum coordinates along each axis
    min_coords = np.min(point_cloud, axis=0)
    max_coords = np.max(point_cloud, axis=0)

    # Define the bounding box using the min and max coordinates
    bounding_box_min = [str(i) for i in list(min_coords[:3])]
    bounding_box_max = [str(i) for i in list(max_coords[:3])]

if __name__ == "__main__":
    rospy.init_node("create_ROI")
    point_cloud_topic = "rviz_selected_points"  # Replace with your actual topic name
    rospy.Subscriber(point_cloud_topic, PointCloud2, point_cloud_callback)

    current_ROI = open("/home/ow-labs/owl_dec/src/owl_description/config/bounding_box.yaml","r")
    data=yaml.load(current_ROI, Loader=BaseLoader)
    if data is None:
        data = {}

    while not rospy.is_shutdown():
        if bounding_box_max is not None and bounding_box_min is not None:
            region_name = input("Enter the name of the region:")
            data[region_name] = {'min':bounding_box_min,'max':bounding_box_max}
            bounding_box_max = None
            bounding_box_min = None

            file=open("/home/ow-labs/owl_dec/src/owl_description/config/bounding_box.yaml","w")
            yaml.dump(data,file,default_flow_style=False)
            file.close()
    current_ROI.close()

        

