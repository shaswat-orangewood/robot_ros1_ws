#! /usr/bin/env python3

import rospy
from visualization_msgs.msg import Marker,MarkerArray
from geometry_msgs.msg import Point
import yaml
from yaml.loader import BaseLoader
i = 0
text_marker_list = []
marker_list = []
def publish_bounding_box_marker(data,marker_pub):
    global i
    global text_marker_list, marker_list

    for key,value in data.items():
        
        marker_name = key
        if len(text_marker_list) != 0 and marker_name in text_marker_list:
            continue
        bounding_box_max = [float(i) for i in value['max']]
        bounding_box_min = [float(i) for i in value['min']]
        bounding_box_center = [(bounding_box_max[i] + bounding_box_min[i])/2 for i in range(3)]
        bounding_box_size = [bounding_box_max[i] - bounding_box_min[i] for i in range(3)]
        marker = Marker()
        marker.header.frame_id = "base_link"  
        marker.ns = "bounding_box"+str(i)
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = bounding_box_center[0]  
        marker.pose.position.y = bounding_box_center[1]
        marker.pose.position.z = bounding_box_center[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = bounding_box_size[0]  
        marker.scale.y = bounding_box_size[1]
        marker.scale.z = bounding_box_size[2]
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.5  # Adjust transparency

        text_marker = Marker()  
        text_marker.header.frame_id = "base_link"  
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.ns = "text_box"+str(i)
        text_marker.pose.position.x = bounding_box_max[0] 
        text_marker.pose.position.y = bounding_box_max[1]
        text_marker.pose.position.z = bounding_box_max[2] 
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.1  # Font size
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0  # Solid color
        text_marker.text = marker_name  
        text_marker_list.append(marker_name)

        i += 1
        marker_list.append(marker)
        marker_list.append(text_marker)
    
    rate = rospy.Rate(1)  # Publish once per second
    for i in range(len(marker_list)):
        marker_list[i].header.stamp = rospy.Time.now()

    marker_pub.publish(marker_list)
    rate.sleep()

if __name__ == "__main__":
    
    rospy.init_node("bounding_box_visualization")
    marker_pub = rospy.Publisher("bounding_box_marker", MarkerArray, queue_size=10)
    
    while not rospy.is_shutdown():
        try:
            current_ROI = open("/home/ow-labs/owl_dec/src/owl_description/config/bounding_box.yaml","r")
            data=yaml.load(current_ROI, Loader=BaseLoader)
            publish_bounding_box_marker(data,marker_pub)
        except rospy.ROSInterruptException:
            pass
