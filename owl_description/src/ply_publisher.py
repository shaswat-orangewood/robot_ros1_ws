#!/usr/bin/env python3

import rospy
import numpy as np
import open3d as o3d
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import struct

def fit_circle(points):

    cenroid = np.mean(points,axis = 0)
    farthest_point = points[np.argmax(np.linalg.norm(points - cenroid,axis = 1))]
    circle_centre = (cenroid +  farthest_point)/2

    return circle_centre

def transform_to_ground_parallel(pcd):
    # Use RANSAC plane segmentation to find the plane
    plane_model, inliers = pcd.segment_plane(distance_threshold=0.01, ransac_n=3, num_iterations=1000)
    [a, b, c, d] = plane_model
    print(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

    # The ground plane normal vector
    ground_normal = np.array([a, b, c])
     
    # The reference normal vector (z-axis)
    reference_normal = np.array([0, 0, 1])

    # Compute the rotation axis and angle
    axis = np.cross(ground_normal, reference_normal)
    axis = axis / np.linalg.norm(axis)
    angle = np.arccos(np.dot(ground_normal, reference_normal))

    # Compute the rotation matrix
    R = o3d.geometry.get_rotation_matrix_from_axis_angle(axis * angle)
    rz = np.array([[np.cos(np.pi/2), -np.sin(np.pi/2), 0], [np.sin(np.pi/2), np.cos(np.pi/2), 0], [0, 0, 1]])
    rx = np.array([[1, 0, 0], [0, np.cos(np.pi), -np.sin(np.pi)], [0, np.sin(np.pi), np.cos(np.pi)]])
    R = rz @ rx @ R
    # Apply the transformation to the point cloud
    transformation = np.eye(4)
    transformation[:3, :3] = R
    pcd.transform(transformation)

    points = np.asarray(pcd.points)
    mid_z = np.median(points[:,2])
    top_points = points[points[:,2] > mid_z]
    bottom_points = points[points[:,2] <= mid_z]


    # top_circle = fit_circle(top_points[:,:2])
    # bottom_circle = fit_circle(bottom_points[:,:2])

    # print(f"the top center is ({top_circle[0]},{top_circle[1]})")
    # print(f"the bottom center is ({bottom_circle[0]},{bottom_circle[1]})")

    # center_x = (top_circle[0] + bottom_circle[0])/200
    # center_y = (top_circle[1] + bottom_circle[1])/200
    
    # print(f"the center is ({center_x},{center_y})")

    min_z = mid_z/100
    min_x = 0.4
    min_y = -0.4

    # Translate the point cloud upward by this minimum z-value
    translation_transformation = np.eye(4)
    translation_transformation[0, 3] = min_x
    translation_transformation[1, 3] = min_y
    translation_transformation[2, 3] = min_z
    pcd.transform(translation_transformation)

    return pcd


def ply_to_pointcloud2(pcd):
    # Load the .ply file using open3d
    points = np.asarray(pcd.points)
    # Convert to PointCloud2
    header = rospy.Header()
    header.stamp = rospy.Time.now()
    header.frame_id = 'base_link'  # Change to your required frame
    
    # Define the fields for the PointCloud2 message
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
    ]

    return pc2.create_cloud(header, fields, points)

def main():
    rospy.init_node('ply_publisher')
    
    # Define the publisher
    pub = rospy.Publisher('/point_cloud', PointCloud2, queue_size=10)
    
    rate = rospy.Rate(1)  # 1Hz for demonstration; adjust as needed
    ply_file = "/home/ow-labs/Documents/RTAB-Map/final_map.ply"  # Change to your file's path
    pcd = o3d.io.read_point_cloud(ply_file)
    # pcd = pcd.voxel_down_sample(voxel_size = 0.05)
    cl,ind = pcd.remove_statistical_outlier(nb_neighbors = 20, std_ratio = 1.0)
    pcd = pcd.select_by_index(ind)

    pcd = transform_to_ground_parallel(pcd)
    
    while not rospy.is_shutdown():

        point_cloud = ply_to_pointcloud2(pcd)
        pub.publish(point_cloud)
        rate.sleep()

if __name__ == '__main__':
    main()
