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
import tf2_ros
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion
import tf2_geometry_msgs
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

FRAME_ID = 'base_link'
SCENE = moveit_commander.PlanningSceneInterface()
(X, Y, Z, W) = (0, 1, 2, 3)

def create_collision_object(id, dimensions, pose):
    object = CollisionObject()
    object.id = id
    object.header.frame_id = FRAME_ID

    solid = SolidPrimitive()
    solid.type = solid.BOX
    solid.dimensions = dimensions
    object.primitives = [solid]

    object_pose = Pose()
    object_pose.position.x = pose[X]
    object_pose.position.y = pose[Y]
    object_pose.position.z = pose[Z]

    object.primitive_poses = [object_pose]
    object.operation = object.ADD
    return object

def add_collision_objects():
    floor_limit = create_collision_object(id='floor_limit',
                                          dimensions=[10, 10, 0.2],
                                          pose=[0, 0, -0.2])
    # table_1 = create_collision_object(id='table_1',
    #                                   dimensions=[0.3, 0.6, 0.2],
    #                                   pose=[-0.7, 0.3, 0.1])
    # table_2 = create_collision_object(id='table_2',
    #                                   dimensions=[0.3, 0.3, 0.2],
    #                                   pose=[-0.25, 0.45, 0.1])
    # target_1 = create_collision_object(id='target_1',
    #                                    dimensions=[0.02, 0.02, 0.2],
    #                                    pose=[0.0, -0.9, 0.3])

    SCENE.add_object(floor_limit)
    # SCENE.add_object(target_1)

def reach_pose(arm, pose, tolerance=0.03):
    arm.set_pose_target(pose)
    # arm.set_position_target(pose)
    arm.set_goal_position_tolerance(tolerance)
    success = arm.go(wait=True)

    arm.stop()
    # return arm.set_pose_target(pose)


def move_to_pose(robot_pose,arm):

    pose = Pose()
    position = robot_pose["position"]
    ort = robot_pose["approach"]
    xaxis = np.array(robot_pose["approach"]).reshape(-1,1)
    yaxis = np.array(robot_pose["binormal"]).reshape(-1,1)
    zaxis = np.array(robot_pose["axis"]).reshape(-1,1)
    # Rmat = np.concatenate((-yaxis,zaxis,-xaxis),axis=1)
    Rmat = np.concatenate((xaxis,yaxis,zaxis),axis=1)

    Rmat = np.concatenate((Rmat,np.array([[0,0,0]])),axis=0)
    Rmat = np.concatenate((Rmat,np.array([[0],[0],[0],[1]])),axis=1)
    print(Rmat)

    orientation = quaternion_from_matrix(Rmat)
    pose.position.x = position[0]
    pose.position.y = position[1]
    pose.position.z = position[2]
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]

    tf_buffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tf_buffer)

    pose_stamped = tf2_geometry_msgs.PoseStamped()
    pose_stamped.pose = pose
    pose_stamped.header.frame_id = "camera_color_optical_frame"
    pose_stamped.header.stamp = rospy.Time(0)

    try:
        # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
        output_pose_stamped = tf_buffer.transform(pose_stamped, "base_link", rospy.Duration(0.1))

    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        raise

    # pose.position.x =  0.04118941124381019
    # pose.position.y = -0.6093283639635318
    # pose.position.z =  0.03733176310254749
    # pose.orientation.x = -0.2549271738090864
    # pose.orientation.y = -0.5761195557782327
    # pose.orientation.z = 0.6993556339255759
    # pose.orientation.w = -0.3376389947860891
    print(output_pose_stamped.pose)
    reach_pose(arm, output_pose_stamped.pose)

if __name__ == "__main__":

    rospy.init_node("Explore_ROI")

    grasp = {}

    file = open("/home/ow-labs/workspaces/gpd/results/grasp.txt","r")
    max_score = -np.inf
    max_index = None
    for text in file.readlines():
        if "grasp" in text:
            index = int(text.split(" : ")[1][:-1])
            grasp[index] = {}
        
        if "position" in text:
            arr = text.split(" : ")[1][:-2]
            grasp[index]["position"] = list(map(float,arr.split(",")))

        if "binormal" in text:
            arr = text.split(" : ")[1][:-2]
            grasp[index]["binormal"] = list(map(float,arr.split(",")))

        if "approach" in text:
            arr = text.split(" : ")[1][:-2]
            grasp[index]["approach"] = list(map(float,arr.split(",")))

        if "axis" in text:
            arr = text.split(" : ")[1][:-2]
            grasp[index]["axis"] = list(map(float,arr.split(",")))

        if "score" in text:
            score = text.split(" : ")[1][:-1]

            grasp[index]["score"] = float(score)

            if float(score) > max_score:
                max_score = float(score)
                max_index = index

    print(max_index)
    print(grasp)

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
    # add_collision_objects()
    move_to_pose(grasp[max_index],arm) 