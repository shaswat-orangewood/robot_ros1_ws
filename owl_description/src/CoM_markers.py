#! /usr/bin/env python

import rospy
from visualization_msgs.msg import Marker 

rospy.init_node('rviz_marker')

marker_pub = rospy.Publisher("/visualization_marker", Marker, queue_size = 2)


marker = Marker()

marker.header.frame_id = "link1"
marker.frame_locked = True
marker.header.stamp = rospy.Time.now()

# set shape, Arrow: 0; Cube: 1 ; Sphere: 2 ; Cylinder: 3
marker.type = 2
marker.id = 0

# Set the scale of the marker 
marker.scale.x = 0.1
marker.scale.y = 0.1
marker.scale.z = 0.1

# Set the color
marker.color.r = 0.0
marker.color.g = 1.0
marker.color.b = 0.0
marker.color.a = 1.0

# Set the pose of the marker    0.00042134 0.034391 -0.10382
# marker.pose.position.x = 02.7395E-05
# marker.pose.position.y = -0.01813
# marker.pose.position.z = 4.3722E-05

marker.pose.position.x = 0.17795 
marker.pose.position.y = 0.00011012
marker.pose.position.z = -0.085428
marker.pose.orientation.x = 0.0
marker.pose.orientation.y = 0.0
marker.pose.orientation.z = 0.0
marker.pose.orientation.w = 1.0

while not rospy.is_shutdown():
  marker_pub.publish(marker)
  rospy.rostime.wallsleep(1.0)