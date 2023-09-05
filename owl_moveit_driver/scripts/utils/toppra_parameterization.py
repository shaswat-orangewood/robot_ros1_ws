from __future__ import print_function

# Toppra imports
import toppra as ta
import toppra.constraint as constraint
import toppra.algorithm as algo
import numpy as np
import matplotlib.pyplot as plt
import time
import copy

import rospy
from rospy import Duration
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import matplotlib.pyplot as plt

class ToppraTrajectory:
    def __init__(self, samplingFrequency=250, maxVelocity=0.87, n_gridpoints=0):
        self.joint_names = ["BJ", "SJ", "EJ", "W1J", "W2J", "W3J"]
        self.sampling_frequency = samplingFrequency
        self.n_grid_points = n_gridpoints
        self.max_allowed_velocity = maxVelocity

    def generateToppraTrajectory(self, trajectory: JointTrajectory) -> JointTrajectory:
        rospy.loginfo("Generating TOPP-RA trajectory.")
 
        dof = len(trajectory.points[0].positions)
        n = len(trajectory.points)
        if n <= 1 or dof == 0:
            raise Exception("You must provide at least 2 points to generate a valid trajectory.")

        trajectoryMsg = self.reorgraniseTrajectory(trajectory)
        velocity_scaling = rospy.get_param("velocity_scaling", 0.3)

        # Generate trajectory.
        # create waypoints of n x dof
        way_pts = np.zeros([n, dof])
        for i in range(0, n):
            for j in range(0, dof):
                way_pts[i][j] = trajectoryMsg.points[i].positions[j]

        # Part of TOPP-RA is to generate path(s \in [0,1]) from n waypoints.
        # The algorithm then parametrizes the initial path.
        path = ta.SplineInterpolator(np.linspace(0, 1, n), way_pts)

        # Create velocity and acceleration bounds. Supposing symmetrical bounds around zero.
        vel_limits = np.array([[-self.max_allowed_velocity, self.max_allowed_velocity]] * dof) * velocity_scaling
        pc_vel = constraint.JointVelocityConstraint(vel_limits)
        
        # setup a parametrization instance
        if self.n_grid_points <= 0:
            num_grid_points = np.max([100, n * 2])
            gridpoints = np.linspace(0, path.duration, num_grid_points)
        else:
            gridpoints = np.linspace(0, path.duration, self.n_grid_points)

        instance = algo.TOPPRA([pc_vel], path, solver_wrapper="seidel")

        # Retime the trajectory, only this step is necessary.
        jnt_traj = instance.compute_trajectory(0, 0)

        # Convert to JointTrajectory message
        return self.TOPPRA2JointTrajectory(jnt_traj, self.sampling_frequency)
    
    def reorgraniseTrajectory(self, trajectory:JointTrajectory):
        _joint_sequence = trajectory.joint_names
        _joint_index_map = {}
        for name in _joint_sequence:
            _joint_index_map[name] = _joint_sequence.index(name)
            
        reorganisedTrajectory = JointTrajectory()
        reorganisedTrajectory.header = trajectory.header
        
        for point in trajectory.points:
            traj_point = JointTrajectoryPoint()
            for joint_name in self.joint_names:
                traj_point.positions.append(point.positions[_joint_index_map[joint_name]])
                traj_point.velocities.append(point.velocities[_joint_index_map[joint_name]])
                traj_point.accelerations.append(point.accelerations[_joint_index_map[joint_name]])
            traj_point.time_from_start = point.time_from_start
            reorganisedTrajectory.points.append(traj_point)
            
        return reorganisedTrajectory
         
    def TOPPRA2JointTrajectory(self, jnt_traj, f):
        # Sampling frequency is required to get the time samples correctly.
        # The number of points in ts_sample is duration*frequency.
        ts_sample = np.linspace(0, jnt_traj.get_duration(), int(jnt_traj.get_duration() * f))
        
        # Sampling. This returns a matrix for all DOFs. Accessing specific one is
        # simple: qs_sample[:, 0]
        qs_sample = jnt_traj.eval(ts_sample)
        qds_sample = jnt_traj.evald(ts_sample)
        qdds_sample = jnt_traj.evaldd(ts_sample)

        n = qs_sample.shape[0]
        dof = qs_sample.shape[1]

        # Transform into JointTrajectory
        joint_trajectory = JointTrajectory()
        for i in range(0, n):
            temp_point = JointTrajectoryPoint()

            for j in range(0, dof):
                temp_point.positions.append(qs_sample[i, j])
                temp_point.velocities.append(qds_sample[i, j])
                temp_point.accelerations.append(qdds_sample[i, j])

            temp_point.time_from_start = Duration.from_sec(i / f)
            joint_trajectory.points.append(temp_point)

        # Add last point with zero velocity and acceleration
        last_point = JointTrajectoryPoint()
        for i in range(0, dof):
            last_point.positions.append(qs_sample[n - 1, i])
            last_point.velocities.append(0.0)
            last_point.accelerations.append(0.0)
        last_point.time_from_start = Duration.from_sec(n / f)
        joint_trajectory.points.append(last_point)
        joint_trajectory.joint_names = self.joint_names

        return joint_trajectory
