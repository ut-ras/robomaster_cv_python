## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2.pyrealsense2 as rs
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

    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

    if device_product_line == 'D435i':
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    else:
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Start streaming
    pipeline.start(config)

def get_color_depth_image():
    try:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        #convert frames to images
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        return color_image, depth_image
    except:
        pipeline.stop()

def set_all_bounding_box_depth_values(depth_image, box_list):
    if box_list == None or len(box_list) == 0:
        return
    else:
        for i in range(len(box_list)):
            depth_value = get_depth_value_from_bounding_box(depth_image, box_list[i])
            box_list[i].set_depth(depth_value)    

#acquiring the depth frame, convert to numpy array so that it can be indexed into by a bounding box
#helper function for set_all_bounding_box_depth_values()
def get_depth_value_from_bounding_box(depth_image, bounding_box):
    try:
        #return a float by indexing into the numpy array using the coordinates given by the bounding box
        y_value = int(bounding_box.get_y_value())
        x_value = int(bounding_box.get_x_value())
        depth_value = depth_image[y_value,x_value]
        return depth_value
    except:
        #stop streaming
        pipeline.stop()

if __name__ == "__main__":
<<<<<<< HEAD
    initialize_real_sense()
    # get_color_image()
    # get_depth_at_pixel(0)
=======
    initialize_real_sense()
>>>>>>> 9d6db8576d6d73ed838c921b84d40029d649d6e8
