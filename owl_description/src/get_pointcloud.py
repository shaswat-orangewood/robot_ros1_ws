import pyrealsense2 as rs
import pcl
import numpy as np
import torch
import clip
import ssl
import cv2
import math
import json
import time
import numpy as np
from PIL import Image, ImageDraw, ImageFont
from transformers import pipeline

model, preprocess = clip.load("ViT-B/32",device="cpu")
model.eval()

print("ZERO SHOT WORKING")
# Initialize the object detection model
checkpoint = "google/owlvit-base-patch32"
detector = pipeline(model=checkpoint, task="zero-shot-object-detection", device="cpu")

# Configure depth and color streams
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start the RealSense pipeline
pipeline1 = rs.pipeline()
profile = pipeline1.start(config)

pc = rs.pointcloud()
# Create a PCL PointCloud object
pcl_cloud = pcl.PointCloud()

# Wait for a coherent pair of frames
frames = pipeline1.wait_for_frames()
hole_filling = rs.hole_filling_filter(1)

align = rs.align(rs.stream.color)  
aligned_frames = align.process(frames)
color_frame = aligned_frames.get_color_frame()
depth_frame = aligned_frames.get_depth_frame()
pc.map_to(color_frame)
depth_mod = hole_filling.process(depth_frame)
points = pc.calculate(depth_mod)
point_cloud = np.asanyarray(points.get_vertices())
point_cloud = np.asanyarray(point_cloud).view(np.float32)


color_image = np.asanyarray(color_frame.get_data())
depth_image = np.asanyarray(depth_frame.get_data())

color_frame_copy = np.copy(color_image)
depth_frame_copy = np.copy(depth_image)
print(depth_frame_copy.shape)
pointcloud_copy = np.copy(point_cloud).reshape(depth_frame_copy.shape[0],depth_frame_copy.shape[1],3)


clm = depth_frame_copy.shape[1]
row = depth_frame_copy.shape[0]

# color_frame_copy = cv2.rotate(color_frame_copy,cv2.ROTATE_180)
image = Image.fromarray(cv2.cvtColor(color_frame_copy, cv2.COLOR_BGR2RGB))
# image.save("test.jpg")
detection_results = {}
predictions = detector(image, candidate_labels=["box","book","spray can"])
print(predictions)
draw = ImageDraw.Draw(image)
i = 0
for prediction in predictions:
    
    box = prediction["box"]
    xmin, ymin, xmax, ymax = box.values()
    
    # run clip on the cropped image
    cropped_image = image.crop((xmin, ymin, xmax, ymax))
    cropped_image = preprocess(cropped_image).unsqueeze(0).to("cpu")
    labels = ["box","book","can", "spray"]
    text = clip.tokenize(labels).to("cpu")

    with torch.no_grad():
        logits_per_image, logits_per_text = model(cropped_image, text)
        probs = logits_per_image.softmax(dim=-1).cpu().numpy()

    # overlay the predicted label on the image
    best_label = labels[probs.argmax()]

    # make the text bolder
    cv2.rectangle(color_frame_copy, (xmin, ymin), (xmax, ymax), 0, 2, 0)
    cv2.putText(color_frame_copy, f"{best_label}", (xmin, ymin), cv2.FONT_HERSHEY_COMPLEX, 1, 0, 2, 1)

    center = (int((xmin+xmax)/2), int((ymin+ymax)/2))
    area = abs(xmax-xmin)*abs(ymax-ymin)

    depth = int(np.average(depth_frame_copy[ymin:ymax,xmin:xmax]))
    
    if math.isnan(depth):
        if 0 <= center[0] < clm and 0 <= center[1] < row: 
            depth = int(depth_frame_copy[center[1],center[0]])
        else:
            print("WARNING : OBJECT NOT FOUND")
            continue
    else:

        if 0 <= xmin < clm and 0 <= xmax < clm and 0 <= ymin < row and 0 <= ymax < row: 
            depth = int(np.average(depth_frame_copy[ymin:ymax,xmin:xmax])) # Averaging the depth over object bounding box
        else:
            print("WARNING : OBJECT NOT FOUND")
            continue

    # pointcloud_object = pointcloud_copy[ymin:ymax,xmin:xmax,:].reshape(-1,3)
    # pcl_cloud.from_array(pointcloud_object)
pointcloud_object = pointcloud_copy.reshape(-1,3)
pcl_cloud.from_array(pointcloud_object)
pcl.save(pcl_cloud, '/home/ow-labs/workspaces/gpd/tutorials/pointcloud.pcd')
    # print(probs)
    # detection_result = {"detected_object": f"{best_label}", "confidence": 100, "center_pt": center, "depth": depth,"area":area}
    # # print(detection_result)
    # s_no = "detection_" + str(i)
    # detection_results[s_no] = detection_result
    # i = i+1
    # draw.text((xmin, ymin), f"{best_label}", fill="red", font=font)
    # draw.rectangle((xmin, ymin, xmax, ymax), outline="green", width=2)
# Display the resulting frame