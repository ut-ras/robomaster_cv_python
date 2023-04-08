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

#the depth image resolution is set to 1280 x 720p, USB 3.0 required to access 1280 by 720 otherwise, crashes
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

    config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)

    if device_product_line == 'D435i':
        config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

#testing purposes
def get_color_image():
    try:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        color_image = np.asanyarray(color_frame.get_data())

        return color_image
    except:
        pipeline.stop()
    # return color_image
#acquiring the depth frame, convert to numpy array so that it can be indexed into by a bounding box
def get_depth_value_from_bounding_box(bounding_box):
    try:
        
        frames = pipeline.wait_for_frames()

        #acquire the depth frame
        depth_frame = frames.get_depth_frame()

        #convert depth frame into numpy array of shape 720 by 1280
        depth_frame = np.asanyarray(depth_frame.get_data())

        #return a float by indexing into the numpy array using the coordinates given by the bounding box
        return depth_frame[bounding_box.get_y_value(),bounding_box.get_x_value()]
    except:
        #stop streaming
        pipeline.stop()

def set_all_bounding_box_depth_values(box_list):
    if box_list == None or len(box_list) == 0:
        return
    else:
        for i in range(len(box_list)):
            box_list[i].set_depth(get_depth_value_from_bounding_box(box_list[i]))    

if __name__ == "__main__":
    initialize_real_sense()