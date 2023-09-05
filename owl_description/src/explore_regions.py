#! /usr/bin/env python3

import rospy
import yaml
import numpy as np
import moveit_commander
from tf.transformations import quaternion_from_euler,quaternion_multiply,quaternion_from_matrix
from geometry_msgs.msg import Pose
import sys
from yaml.loader import BaseLoader
import numpy as np
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion
# Rmat_home = np.array([[0, -1, 0], [-1, 0, 0], [0, 0, -1]])
Rmat_home = np.array([[1, 0, 0,0], [0, 1, 0,0], [0, 0, 1,0],[0,0,0,1]])

SCENE = moveit_commander.PlanningSceneInterface()

def get_rotation_matrix(vec2, vec1=np.array([0, 0, 1])):
    """get rotation matrix between two vectors using scipy"""
    vec1 = np.reshape(vec1, (1, -1))
    vec2 = np.reshape(vec2, (1, -1))
    direction = vec2 / np.linalg.norm(vec2)

    # Calculate the rotation matrix
    z_axis = np.array([0.0, 0.0, 1.0])  # Assuming your world frame has Z-axis pointing up
    rotation_matrix = np.eye(3) + np.cross(z_axis, direction) @ Rmat_home
    # rot_home = R.from_matrix(Rmat_home)
    # Convert the rotation matrix to a Rotation object
    r = R.from_matrix(rotation_matrix)
    return r.as_euler('xyz',degrees=False)



def reach_pose(arm, pose, tolerance=0.03):
    arm.set_pose_target(pose)
    # arm.set_position_target(pose)
    arm.set_goal_position_tolerance(tolerance)
    success = arm.go(wait=True)

    arm.stop()
    # return arm.set_pose_target(pose)


def move_to_pose(robot_pose,arm,centre):

    pose = Pose()
    pose.position.x = robot_pose[0]
    pose.position.y = robot_pose[1]
    pose.position.z = robot_pose[2]

    if len(robot_pose) == 6:
        orientation = quaternion_from_euler(robot_pose[3],robot_pose[4],robot_pose[5])
    else:
        # vec =  np.array(robot_pose) - np.array(centre)
        # rotation = arm.get_current_rpy()
        # target = [robot_pose[i] - centre[i] for i in range(len(centre))]
        # target = target/np.linalg.norm(target)
        # obj_roll = np.arccos(target[0])
        # obj_pitch = np.arccos(target[1])
        # obj_yaw = np.arccos(target[2])
        # pot_rotation = quaternion_from_euler(obj_roll, obj_pitch, obj_yaw)
        # quat_ee = quaternion_from_euler(rotation[0], rotation[1], rotation[2])
        # orientation = quaternion_multiply(quat_ee, pot_rotation)
        # orientation = quaternion_multiply(orientation, pot_rotation)
        # roll,pitch,yaw = get_rotation_matrix(vec)
        # roll,pitch,yaw = -3.0432678643284596, -0.5858272756145722, 1.5277854454279125
        roll,pitch,yaw = 3.047080325315432, -0.5793243021751122, 1.554287754605568
        # roll,pitch,yaw = 3.1415927, 0.0, 1.5707963
        # print(f"the RPY angles are : {roll},{pitch},{yaw}")
        orientation = quaternion_from_euler(roll,pitch,yaw)
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    reach_pose(arm, pose)

if __name__ == "__main__":

    rospy.init_node("Explore_ROI")

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.sleep(2)

    arm = moveit_commander.MoveGroupCommander('owl_arm',
                                              ns=rospy.get_namespace())
    # robot = moveit_commander.RobotCommander('robot_description')
    # gripper = robot.get_joint('gripper_finger1_joint')

    arm.set_num_planning_attempts(90)
    arm.set_planning_time(20)
    arm.set_planner_id("RRT")
    arm.set_goal_position_tolerance(0.5)

    current_ROI = open("/home/ow-labs/owl_dec/src/owl_description/config/bounding_box.yaml","r")
    data=yaml.load(current_ROI, Loader=BaseLoader)

    # start_region = input("Enter the name of the start region for exploration:")
    start_region = "test_spary_can"

    bounding_box = data[start_region]

    bounding_box_max = [float(i) for i in bounding_box['max']]
    bounding_box_min = [float(i) for i in bounding_box['min']]
    bounding_box_center = [(bounding_box_max[i] + bounding_box_min[i])/2 for i in range(3)]
    print(bounding_box_center)

    bounding_box_size = [bounding_box_max[i] - bounding_box_min[i] for i in range(3)]
    
    expl_region = [bounding_box_center[0],bounding_box_center[1] + 0.3,bounding_box_center[2] + 0.3]
    move_to_pose(expl_region,arm,bounding_box_center)    
    