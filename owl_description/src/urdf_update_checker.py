#!/usr/bin/env python
from __future__ import print_function
import rospy
import subprocess
import os

        
if __name__ == "__main__":
    rospy.init_node('robot_description_checker')
    print("Payload checker")
    payload = 0.0
    while(1):
    
        if payload == rospy.get_param('/payload'):
            rospy.sleep(1)
            continue
        lin_transform = rospy.get_param('/lin_transform')
        rot_transform = rospy.get_param('/rot_transform')
        payload = rospy.get_param('/payload')
        dirname,filename = os.path.split(os.path.abspath(__file__))
        xyz = '{} {} {}'.format(lin_transform[0],lin_transform[1],lin_transform[2])
        rpy = '{} {} {}'.format(rot_transform[0],rot_transform[1],rot_transform[2])
        os.chdir(dirname)
        xacroPath="../urdf_new/urdf_6_3.xacro"
        updatedURDF="../urdf_new/urdf_6_3_updated.urdf"
        command_string = "rosrun xacro xacro {} > {} xyz:='{}' rpy:='{}' pay_load:={}".format(xacroPath, updatedURDF, xyz, rpy, payload)
        try:
            subprocess.run(command_string, shell=True)
            file = open(updatedURDF,"r")
            robot_description = file.read()
            file.close()
            rospy.set_param("/robot_description", robot_description)
            rospy.loginfo("payload updated")
        except subprocess.CalledProcessError as process_error:
            rospy.logerr('Failed to run xacro command with error: \n%s', process_error.output)
        rospy.sleep(1)
