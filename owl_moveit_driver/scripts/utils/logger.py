import rospy
import numpy as np

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import rospkg

class Logger:
    def __init__(self,path=None):
        if not path:
            rospack = rospkg.RosPack()
            path = rospack.get_path('owl_moveit_driver') + '/logs/'
        self.path = path
        self.joint_names = ['BJ', 'SJ', 'EJ', 'W1J', 'W2J', 'W3J']

    def write_joint_positions(self,joint_positions, filename='joint_positions.txt'):

        filename = self.path + filename
        with open(filename, 'w') as file:
            file.write(f"Joint Names - {self.joint_names}\n")

            for pos in joint_positions:
                file.write(' '.join(str(angle) for angle in pos))
                file.write('\n')

    def write_joint_trajectory(self,trajectory_msg, filename='joint_trajectory.txt'):
        num_points = len(trajectory_msg.points)
        joint_names = trajectory_msg.joint_names

        filename = self.path + filename
        with open(filename, 'w') as file:
            file.write(f"JointTrajectory with {num_points} Points\n")
            file.write("Joint Names - ")
            file.write(f"{joint_names}\n\n")

            file.write("Joint Positions\n")
            for point in trajectory_msg.points:
                file.write(f"{point.positions}\n")
            file.write("\n")

            file.write("Joint Velocities\n")
            for point in trajectory_msg.points:
                file.write(f"{point.velocities}\n")
            file.write("\n")

            file.write("Joint Accelerations\n")
            for point in trajectory_msg.points:
                file.write(f"{point.accelerations}\n")
            file.write("\n")


    def plot_waypoints(self,waypoints,joint_positions_raw=None,filename='waypoints.png'):
        '''
        Plot waypoints in 3D. Take x,y,z from waypoints.poses.position
        '''

        # Extract x, y, z coordinates from waypoints
        x = [pose.position.x for pose in waypoints.poses]
        y = [pose.position.y for pose in waypoints.poses]
        z = [pose.position.z for pose in waypoints.poses]

        # Create a 3D plot
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        # Plot waypoints
        if joint_positions_raw is None:
            ax.scatter(x, y, z, c='b', marker='o')
        else:
            for i in range(len(waypoints.poses)):
                if joint_positions_raw[i] is None:
                    ax.scatter(x[i], y[i], z[i], c='red', marker='o')
                else:
                    ax.scatter(x[i], y[i], z[i], c='green', marker='o')

        # Set labels and title
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')
        ax.set_title('Waypoints')

        # Set the desired viewpoint
        ax.view_init(elev=20, azim=40)

        # Save the plot to a file
        filename = self.path + filename
        plt.savefig(filename)
        plt.close()

        # plt.show()

    def plot_joint_trajectory(self,joint_trajectory,filename='joint_trajectory.png'):
        # Get joint names
        joint_names = joint_trajectory.joint_names

        # Get positions, velocities, and accelerations for each joint
        positions = []
        velocities = []
        accelerations = []
        times = []
        for point in joint_trajectory.points:
            positions.append(point.positions)
            velocities.append(point.velocities)
            accelerations.append(point.accelerations)
            times.append(point.time_from_start.to_sec())

        # Create subplots for position, velocity, and acceleration
        fig, axs = plt.subplots(3, 1, figsize=(15, 15))
        fig.suptitle('Joint Trajectory')

        # Plot position for each joint
        axs[0].set_title('Position')
        axs[0].set_xlabel('Time (s)')
        axs[0].set_ylabel('Position (rad)')
        for i in range(6):
            axs[0].plot(times, [p[i] for p in positions], label=joint_names[i])
        axs[0].legend()

        # Plot velocity for each joint
        axs[1].set_title('Velocity')
        axs[1].set_xlabel('Time (s)')
        axs[1].set_ylabel('Velocity (rad/s)')
        for i in range(6):
            axs[1].plot(times, [v[i] for v in velocities], label=joint_names[i])
        axs[1].legend()

        # Plot acceleration for each joint
        axs[2].set_title('Acceleration')
        axs[2].set_xlabel('Time (s)')
        axs[2].set_ylabel('Acceleration (rad/s^2)')
        for i in range(6):
            axs[2].plot(times, [a[i] for a in accelerations], label=joint_names[i])
        axs[2].legend()

        # Save the plot to a file
        filename = self.path + filename
        plt.savefig(filename)
        plt.close()