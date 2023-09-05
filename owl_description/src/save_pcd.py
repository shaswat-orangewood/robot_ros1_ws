import pyrealsense2 as rs
import pcl
import numpy as np

# Configure depth and color streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the RealSense pipeline
pipeline = rs.pipeline()
profile = pipeline.start(config)

pc = rs.pointcloud()
# Create a PCL PointCloud object
pcl_cloud = pcl.PointCloud()

# Wait for a coherent pair of frames
frames = pipeline.wait_for_frames()
hole_filling = rs.hole_filling_filter(1)

align = rs.align(rs.stream.color)  
aligned_frames = align.process(frames)
color_frame = aligned_frames.get_color_frame()
depth_frame = aligned_frames.get_depth_frame()
pc.map_to(color_frame)
depth_mod = hole_filling.process(depth_frame)
points = pc.calculate(depth_mod)
vtx = np.asanyarray(points.get_vertices())
print(vtx.shape)
verts = np.asanyarray(vtx).view(np.float32).reshape(-1, 3)

pcl_cloud.from_array(verts)

pcl.save(pcl_cloud, 'pointcloud.pcd')