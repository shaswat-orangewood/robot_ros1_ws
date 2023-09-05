#!/usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg

if __name__ == '__main__':
    rospy.init_node('static_tf2_broadcaster')
    static_broadcaster = tf2_ros.StaticTransformBroadcaster()

    static_transformStamped = geometry_msgs.msg.TransformStamped()

    static_transformStamped.header.stamp = rospy.Time.now()
    static_transformStamped.header.frame_id = "map"
    static_transformStamped.child_frame_id = "///map"

    # # Set the transform here. I'm setting it to identity for simplicity
    # trans = [-0.08638277141676313, 0.02871048090415014, -0.041915580982770234]
    # rot = [0.5000000000000001, -0.5, -0.5007956923220132,0.49920303940552624]
    trans = [0,0,0]
    rot = [0,0,0,1]
    static_transformStamped.transform.translation.x = trans[0]
    static_transformStamped.transform.translation.y = trans[1]
    static_transformStamped.transform.translation.z = trans[2]

    static_transformStamped.transform.rotation.x = rot[0]
    static_transformStamped.transform.rotation.y = rot[1]
    static_transformStamped.transform.rotation.z = rot[2]
    static_transformStamped.transform.rotation.w = rot[3]

    static_broadcaster.sendTransform(static_transformStamped)
    rospy.spin()