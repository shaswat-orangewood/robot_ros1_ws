#!/usr/bin/env python3
import sys
import rospy
from trajectory_msgs.msg import JointTrajectory
from actionlib import SimpleActionServer, GoalStatus
from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
    FollowJointTrajectoryResult,
)
from sensor_msgs.msg import JointState
from owl_client import OwlClient , Trajectory
import time

from utils import ToppraTrajectory , Logger

class OWLDriver:
    def __init__(self):
        rospy.init_node("OWL_driver")
        try:
            _robot_ip = "10.42.0.52"
        except:
            rospy.logwarn("Unable to get the robot_ip...")
            sys.exit(-1)
        self.client = OwlClient(_robot_ip)
        self.logger = Logger()
        while not self.client.is_running():
            if rospy.is_shutdown():
                sys.exit(-1)
            rospy.logwarn("Trying to connect to OWLBot...")
            time.sleep(0.2)
        rospy.loginfo("Robot ip: {0}".format(_robot_ip))
        self._action_server = SimpleActionServer(
            "owl_arm_controller/follow_joint_trajectory",
            FollowJointTrajectoryAction,
            execute_cb=self.execute_callback,
            auto_start=False,
        )
        self.toppraTrajectory = ToppraTrajectory()
        self.publisher = rospy.Publisher("/joint_states", JointState, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(0.01), self.joint_state_publisher)
        rospy.on_shutdown(self.close)

    def cancel_callback(self):
        """Accept or reject a client request to cancel an action."""
        self.client.move_abort()

    def execute_callback(self, goal_handle):
        rospy.loginfo("Executing goal...")

        if self._action_server.is_preempt_requested():
            rospy.loginfo("Cancel request received...")
            self._action_server.set_preempted()
            return
               
        sampled_trajectory = self.toppraTrajectory.generateToppraTrajectory(goal_handle.trajectory)
        # self.logger.plot_joint_trajectory(sampled_trajectory)
        try:
            self.client.move_trajectory(self.getTrajectory(sampled_trajectory))
        except Exception as e:
            rospy.logwarn(str(e.details()))

            if self._action_server.is_preempt_requested():
                self._action_server.set_preempted()
                return
            else: 
                self._action_server.set_aborted()
                return

        self._action_server.set_succeeded(FollowJointTrajectoryResult())
        rospy.loginfo("Goal executed...")

    def getTrajectory(self, trajectoryMsg : JointTrajectory):
        positions = []
        velocities = []
        accelerations = []
        
        for i in range(len(trajectoryMsg.points)):
            positions.append(trajectoryMsg.points[i].positions)
            velocities.append(trajectoryMsg.points[i].velocities)
            accelerations.append(trajectoryMsg.points[i].accelerations)
        return Trajectory(positions,velocities,accelerations)

    def joint_state_publisher(self, event):
        current_joint = self.client.get_joint()
        joint_state = JointState()
        joint_state.header.stamp = rospy.Time().now()
        joint_state.name = ["BJ", "SJ", "EJ", "W1J", "W2J", "W3J"]
        joint_state.position = [
            current_joint.Base,
            current_joint.Shoulder,
            current_joint.Elbow,
            current_joint.Wrist1,
            current_joint.Wrist2,
            current_joint.Wrist3,
        ]
        self.publisher.publish(joint_state)

    def run(self):
        self._action_server.start()
        rospy.loginfo("OWLDriver is running...")
        rospy.spin()

    def close(self):
        self.client.close()

if __name__ == "__main__":
    owl_driver = OWLDriver()
    owl_driver.run()
