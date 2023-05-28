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

# Configure depth and color streams
config = rs.config()

#the depth image resolution is set to 1280 x 720p, USB 3.0 required to access 1280 by 720 otherwise, crashes
def initialize_real_sense():

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

# Creates align object for depth frame
align_to = rs.stream.color
align = rs.align(align_to)

def get_color_depth_image():
    try:
        frames = pipeline.wait_for_frames()

        # Get aligned frames
        aligned_frames = align.process(frames)

        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()

        #convert frames to images
        color_image = np.asanyarray(color_frame.get_data(), dtype=np.uint8)
        depth_image = np.asanyarray(depth_frame.get_data(), dtype=np.uint16)

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

#index into depth image using coordinates from bounding box
#helper function for set_all_bounding_box_depth_values()
def get_depth_value_from_bounding_box(depth_image, bounding_box):
    #return a float by indexing into the numpy array using the coordinates given by the bounding box
    x1, x2 = bounding_box.get_x_value()
    y1, y2 = bounding_box.get_y_value()

	# cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
    # depth_10000 = depth_image/10000.
    # depth_10000[x1:x2, y1:y2] = 1
    # cv2.rectangle(depth_10000, (x1, y1), (x2, y2), 1, 3)
    # cv2.imshow('Depth image', depth_10000)
    # cv2.waitKey(1)
    # print("xcoordinates", x1, x2)
    # print("ycoordinates", y1, y2)

    #TODO
    #Align the depth image with the color image using align from the python wrapper
    depth_box = depth_image[y1:y2, x1:x2]

    #not using nanmean as nan is not returned for integer data types
    depth_value = np.mean(depth_box[np.nonzero(depth_box)])

    print("Depth value", depth_value)
    return depth_value #To convert to meters

def get_intrinsics():
    profile = config.get_stream(rs.stream.depth)
    intrinsics = profile.as_video_stream_profile().get_intrinsics()
    return intrinsics

if __name__ == "__main__":
    initialize_real_sense()
    # get_color_image()
    # get_depth_at_pixel(0)

