'''
    @file: rgb_depth_videos.py
    @brief: program to get rgb and depth color videos (tested on Ubuntu 20.04 PC & Jetson Nano with Ubuntu 18.04)
    @command: python3 rgb_depth_videos.py
    @author: khang nguyen - rvl
'''
#! usr/bin/python3

# Import neccessary libraries
import cv2
import numpy as np
import pyrealsense2 as rs


# Camera parameters
index, fps, width, height = 0, 20, 640, 480
fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
video_rgb = cv2.VideoWriter('vid_data/video_rgb.mp4', fourcc, fps, (width, height))
video_depth = cv2.VideoWriter('vid_data/video_depth.mp4', fourcc, fps, (width, height))

# Configure depth and color streams
pipeline = rs.pipeline()
config = rs.config()

# Get device product line for setting a supporting resolution
pipeline_wrapper = rs.pipeline_wrapper(pipeline)
pipeline_profile = config.resolve(pipeline_wrapper)
device = pipeline_profile.get_device()

# Check availability of device and device compatibility
found_rgb = False
for s in device.sensors:
    if s.get_info(rs.camera_info.name) == 'RGB Camera':
        found_rgb = True
        break
if not found_rgb:
    print("Depth camera with Color sensor is required!\n")
    exit(0)

# Configure streaming formats for RGB and Depth images then start streaming
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:

        # Wait for a coherent pair of frames: depth and color
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        if not depth_frame or not color_frame:
            continue

        # Convert images to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Apply colormap on depth image (image must be converted to 8-bit per pixel first)
        depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)
        depth_colormap_dim = depth_colormap.shape
        color_colormap_dim = color_image.shape

        # If depth and color resolutions are different, resize color image to match depth image for display
        if depth_colormap_dim != color_colormap_dim:
            resized_color_image = cv2.resize(color_image, dsize=(depth_colormap_dim[1], depth_colormap_dim[0]), interpolation=cv2.INTER_AREA)
            images = np.hstack((resized_color_image, depth_colormap))
        else:
            images = np.hstack((color_image, depth_colormap))

        # Show images
        cv2.imshow('RGB and Depth Images', images)

        # Append RGB and Depth images to video
        index += 1
        print("Capture RGB Image %8d" %(index))
        video_rgb.write(color_image)
        print("Capture Depth Image %8d" %(index))
        video_depth.write(depth_colormap)
        
        # Wait for 1 milisecond (ms)
        cv2.waitKey(1)

finally:
    # Stop streaming
    pipeline.stop()

    # Release videos
    video_rgb.release()
    video_depth.release()