## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2 as rs
import numpy as np
import cv2

# Set up pipeline
pipeline = rs.pipeline()

def initialize_real_sense():
    # Configure depth and color streams
    config = rs.config()

    # Get device product line for setting a supporting resolution
    pipeline_wrapper = rs.pipeline_wrapper(pipeline)
    pipeline_profile = config.resolve(pipeline_wrapper)
    device = pipeline_profile.get_device()
    device_product_line = str(device.get_info(rs.camera_info.product_line))

    found_rgb = False
    for s in device.sensors:
        if s.get_info(rs.camera_info.name) == 'RGB Camera':
            found_rgb = True
            break
    if not found_rgb:
        print("The demo requires Depth camera with Color sensor")
        exit(0)

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'D435i':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

def get_color_image_depth_image():
    try:
        # while True:
        frames = pipeline.wait_for_frames()
        #color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        #return color_frame, depth_frame
        return depth_frame
    except:
        #stop streaming
        pipeline.stop()

#bounding_box input parameter should be of type BoundingBox
def get_depth_at_pixel(depth_frame, bounding_box):
    try:

        # Wait for a coherent pair of frames: depth and color
        # frames = pipeline.wait_for_frames()
        # depth_frame = frames.get_depth_frame()

        # Convert images to numpy arrays
        #depth_image returns an array of depth values, in meters
        depth_image = np.asanyarray(depth_frame.get_data())
        # color_image = np.asanyarray(color_frame.get_data())

        # Get depth of specific pixel at the x and y coordinates from the bounding box 
        return depth_image[bounding_box.get_y_value(), bounding_box.get_x_value()]
    except:
    # Stop streaming
        pipeline.stop()

#TODO: Take in bounding box list and just populate all of them
def get_all_color_image_(boxList):
    #color_image, depth_image = get_color_image_depth_image()
    depth_image = get_color_image_depth_image()
    if boxList == None:
        return
    for i in range(len(boxList)):
        boxList[i].set_depth(get_depth_at_pixel(depth_image, boxList[i]))
    #return color_image

if __name__ == "__main__":
    initialize_real_sense()
    # get_color_image()
    # get_depth_at_pixel(0)