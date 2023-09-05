#!/usr/bin/python3

import sys
import math
import rospy
import moveit_commander

from geometry_msgs.msg import Pose, Point, Quaternion
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive
from moveit_msgs.msg import Constraints, OrientationConstraint

from tf.transformations import quaternion_from_euler

FRAME_ID = 'base_link'
(X, Y, Z, W) = (0, 1, 2, 3)
OPEN = 0.0
CLOSE = 0.6
OBJECT_POSITIONS = {'target_1': [0.05, -0.35, 0.3]}
PICK_ORIENTATION_EULER = [0,0,math.pi/2]
PLACE_ORIENTATION_EULER = [0, -math.pi / 2, -math.pi/2]
SCENE = moveit_commander.PlanningSceneInterface()


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
    target_1 = create_collision_object(id='target_1',
                                       dimensions=[0.02, 0.02, 0.2],
                                       pose=[0.0, -0.9, 0.3])

    SCENE.add_object(floor_limit)
    SCENE.add_object(target_1)
    # SCENE.add_object(pillar)


def reach_named_position(arm, target):
    arm.set_named_target(target)
    return arm.execute(arm.plan(), wait=True)


def reach_pose(arm, pose, tolerance=0.03):
    arm.set_pose_target(pose)
    # arm.set_position_target(pose)
    arm.set_goal_position_tolerance(tolerance)
    return arm.go(wait=True)
    # return arm.set_pose_target(pose)


# def open_gripper(gripper):
#     # print("max bound: ",gripper.max_bound())
#     return gripper.move(gripper.max_bound() * OPEN, True)


# def close_gripper(gripper):
#     gripper.move(gripper.max_bound() * CLOSE, True)


def pick_object(name, arm):
    pose = Pose()
    # pose.position.x = OBJECT_POSITIONS[name][X]
    # pose.position.y = OBJECT_POSITIONS[name][Y] - 0.1
    # pose.position.z = OBJECT_POSITIONS[name][Z]
    pose.position.x = 0.0
    pose.position.y = -0.9
    pose.position.z = 0.3
    orientation = quaternion_from_euler(*PICK_ORIENTATION_EULER)
    pose.orientation.x = orientation[0]
    pose.orientation.y = orientation[1]
    pose.orientation.z = orientation[2]
    pose.orientation.w = orientation[3]
    reach_pose(arm, pose)
    # open_gripper(gripper=gripper)
    # pose.position.y += 0.1
    # reach_pose(arm, pose)
    # close_gripper(gripper=gripper)

    # arm.attach_object(name)

    # x = OBJECT_POSITIONS[name][X]
    # y = OBJECT_POSITIONS[name][Y] - 0.1
    # z = OBJECT_POSITIONS[name][Z]
    # position = [x,y,z]
    # reach_pose(arm, position)
    # open_gripper(gripper=gripper)
    # y += 0.1
    # position = [x,y,z]
    # reach_pose(arm, position)
    # close_gripper(gripper=gripper)
    # arm.attach_object(name)


def place_object(name, arm):
    pose = Pose()
    # pose.position.x = OBJECT_POSITIONS[name][Y]
    # pose.position.y = OBJECT_POSITIONS[name][X]
    # pose.position.z = OBJECT_POSITIONS[name][Z]
    pose.position.x = 0
    pose.position.y = 0.35
    pose.position.z = 0.3
    orientation = quaternion_from_euler(*PLACE_ORIENTATION_EULER)
    pose.orientation.x = 1e-6
    pose.orientation.y = 1e-6
    pose.orientation.z = 1e-6
    pose.orientation.w = 1

    reach_pose(arm, pose)
    # open_gripper(gripper=gripper)
    # reach_pose(arm, pose)
    arm.detach_object(name)

    # x = OBJECT_POSITIONS[name][Y]
    # y = OBJECT_POSITIONS[name][X] - 0.1
    # z = OBJECT_POSITIONS[name][Z]
    # position = [x,y,z]
    # reach_pose(arm, position)
    # open_gripper(gripper=gripper)
    # arm.detach_object(name)


def main():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('owl_pick_place')
    rospy.sleep(2)

    arm = moveit_commander.MoveGroupCommander('owl_arm',
                                              ns=rospy.get_namespace())
    # robot = moveit_commander.RobotCommander('robot_description')
    # gripper = robot.get_joint('gripper_finger1_joint')

    arm.set_num_planning_attempts(90)
    arm.set_planning_time(20)
    arm.set_planner_id("RRT")
    arm.set_goal_position_tolerance(0.5)

    # # Set the desired orientation (quaternion) for fixing rotation about Z-axis
    # target_orientation = Quaternion()
    # target_orientation.x = 0.0
    # target_orientation.y = 0.0
    # target_orientation.z = 0.0
    # target_orientation.w = 1.0

    # # Set up the orientation constraint (fix rotation about Z-axis)
    # orientation_constraint = OrientationConstraint()
    # orientation_constraint.link_name = 'tcp'
    # orientation_constraint.header.frame_id = FRAME_ID

    # orientation_constraint.orientation = target_orientation
    # orientation_constraint.absolute_x_axis_tolerance = 0.1  # Tolerance for the X-axis
    # orientation_constraint.absolute_y_axis_tolerance = 0.1  # Tolerance for the Y-axis
    # orientation_constraint.absolute_z_axis_tolerance = 0.0  # Fixed rotation about the Z-axis
    # orientation_constraint.weight = 1.0

    pick_object(name='target_1', arm=arm)
    # place_object(name='target_1', arm=arm)
    # reach_named_position(arm=arm, target='zeros')


if __name__ == '__main__':
    main()
