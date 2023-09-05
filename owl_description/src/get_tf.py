#!/usr/bin/env python3

import rospy
import tf2_ros
from tf.transformations import euler_from_quaternion,quaternion_matrix,translation_matrix

def get_transform():
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)

    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            # We lookup the transform with target_frame as 'tcp' and source_frame as 'camera_colo_optical_frame'
            trans = tfBuffer.lookup_transform('base_link', 'camera_color_optical_frame', rospy.Time())
            
            trans_x = trans.transform.translation.x
            trans_y = trans.transform.translation.y
            trans_z = trans.transform.translation.z

            rot_x = trans.transform.rotation.x
            rot_y = trans.transform.rotation.y
            rot_z = trans.transform.rotation.z
            rot_w = trans.transform.rotation.w
            print(f"the translation is : {trans_x},{trans_y},{trans_z}")
            print(f"the rotation is : {rot_x}, {rot_y}, {rot_z}, {rot_w}")
            rpy_x,rpy_y,rpy_z = euler_from_quaternion([rot_x,rot_y,rot_z,rot_w])
            print(f"RPY angles are : {rpy_x}, {rpy_y}, {rpy_z}")
            # Convert quaternion to Euler angles for easier understanding
            # rotation_matrix = quaternion_matrix((rot_x,rot_y,rot_z,rot_w))[:3, :3]
            # transformation_matrix = translation_matrix((trans_x,trans_y,trans_z)) @ rotation_matrix

            # return transformation_matrix

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rate.sleep()
            print("Waiting for transform to be published")
            continue

        rate.sleep()

if __name__ == '__main__':
    rospy.init_node('tf2_listener')
    get_transform()
