#!/usr/bin/env python
from __future__ import print_function
import rospy
from arm_controllers.srv import urdf_update, urdf_updateResponse
import subprocess
import os

def callback(req):
    
    lin_transform = req.lin_transform
    rot_transform = req.rot_transform
    payload = req.payload
    resp = urdf_updateResponse()
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
        resp.status = True
    except subprocess.CalledProcessError as process_error:
        rospy.logerr('Failed to run xacro command with error: \n%s', process_error.output)
        resp.status = False
    return resp
        
if __name__ == "__main__":
    rospy.init_node('robot_description_updater')
    rospy.Service('urdf_update_service', urdf_update ,callback)
    print("started")
    rospy.spin()