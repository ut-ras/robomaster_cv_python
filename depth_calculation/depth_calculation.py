## License: Apache 2.0. See LICENSE file in root directory.
## Copyright(c) 2015-2017 Intel Corporation. All Rights Reserved.

###############################################
##      Open CV and Numpy integration        ##
###############################################

import pyrealsense2.pyrealsense2 as rs
import numpy as np
import cv2

class RealSense:
    def __init__(self):
        # Set up pipeline
        self.__pipeline__ = rs.pipeline()
        self.__config__ = rs.config()
        self.__color_frame__ = None
        self.__depth_frame__ = None
        self.__color_image__ = None
        self.__depth_image__ = None
        self.__device__ = None
        self.__pipeline_profile__ = None

    
    def get_color_frame(self):
        return self.__color_frame__
    def get_depth_frame(self):
        return self.__depth_frame__
    def get_color_image(self):
        return self.__color_image__
    def get_depth_image(self):
        return self.__depth_image__
    
    def initialize_real_sense(self):
        # Get device product line for setting a supporting resolution
        pipeline_wrapper = rs.pipeline_wrapper(self.__pipeline__)
        self.__pipeline_profile__ = self.__config__.resolve(pipeline_wrapper)
        self.__device__ = self.__pipeline_profile__.get_device()
        device_product_line = str(self.__device__.get_info(rs.camera_info.product_line))

        found_rgb = False
        for s in self.__device__.sensors:
            if s.get_info(rs.camera_info.name) == 'RGB Camera':
                found_rgb = True
                break
        if not found_rgb:
            print("The demo requires Depth camera with Color sensor")
            exit(0)
        # Configure depth and color streams
        self.__config__.enable_stream(rs.stream.depth, 320, 320, rs.format.z16, 45)

        if device_product_line == 'D435i':
            self.__config__.enable_stream(rs.stream.color, 320, 320, rs.format.bgr8, 45)
        else:
            self.__config__.enable_stream(rs.stream.color, 320, 320, rs.format.bgr8, 45)

        # Start streaming
        self.__pipeline__.start(self.__config__)

    def get_color_depth_image(self):

        frames = self.__pipeline__.wait_for_frames()

        # Get aligned frames
        # aligned_frames = self.__pipeline__.process(frames)

        self.__color_frame__ = frames.get_color_frame()
        self.__depth_frame__ = frames.get_depth_frame()

        #convert frames to images
        self.__color_image__ = np.asanyarray(self.__color_frame__.get_data(), dtype=np.uint8)
        self.__depth_image__ = np.asanyarray(self.__depth_frame__.get_data(), dtype=np.uint16)

# Creates align object for depth frame
# align_to = rs.stream.color
# align = rs.align(align_to)

    def set_all_bounding_box_depth_values(self, box_list):
        if box_list == None or len(box_list) == 0:
            return
        else:
            for i in range(len(box_list)):
                depth_value = self.get_depth_value_from_bounding_box(box_list[i])
                box_list[i].set_depth(depth_value)    

#index into depth image using coordinates from bounding box
#helper function for set_all_bounding_box_depth_values()
    def get_depth_value_from_bounding_box(self, bounding_box):
        #return a float by indexing into the numpy array using the coordinates given by the bounding box
        x1, x2 = bounding_box.get_x_value()
        y1, y2 = bounding_box.get_y_value()

        color_frame = self.__color_frame__
        depth_frame = self.__depth_frame__
        profile = self.__pipeline_profile__

        color_intrinsics = color_frame.profile.as_video_stream_profile().intrinsics
        depth_intrinsics = depth_frame.profile.as_video_stream_profile().intrinsics
        depth_to_color_extrin = depth_frame.get_profile().as_video_stream_profile().get_extrinsics_to(color_frame.get_profile())
        color_to_depth_extrin = color_frame.get_profile().as_video_stream_profile().get_extrinsics_to(depth_frame.get_profile())
        depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        # fov = rs.rs2_fov(color_intrinsics)

        corner_one = rs.rs2_project_color_pixel_to_depth_pixel(
        depth_frame.get_data(), depth_scale,
        0.1, 15,
        depth_intrinsics, color_intrinsics, depth_to_color_extrin, color_to_depth_extrin, (x1, y1))

        corner_two = rs.rs2_project_color_pixel_to_depth_pixel(
        depth_frame.get_data(), depth_scale,
        0.1, 15,
        depth_intrinsics, color_intrinsics, depth_to_color_extrin, color_to_depth_extrin, (x2, y2))

        x1 = int(corner_one[0])
        y1 = int(corner_one[1])

        x2 = int(corner_two[0])
        y2 = int(corner_two[1])
        #TODO
        #Why is this array sometimes empty?
        #Either ignore NaNs or catch edge cases for when NaNs occur
        depth_box = self.__depth_image__[y1:y2, x1:x2]

        #not using nanmean as nan is not returned for integer data types
        depth_value = np.mean(depth_box[np.nonzero(depth_box)])

        #To convert to meters, also casts depth value to np.float32
        depth_value = depth_value/np.float32(1000)

        return depth_value 

    def get_intrinsics(self):
        pipeline_wrapper = rs.pipeline_wrapper(self.__pipeline__)
        pipeline_profile = self.__config__.resolve(pipeline_wrapper)
        profile = pipeline_profile.get_stream(rs.stream.depth)
        intrinsics = profile.as_video_stream_profile().get_intrinsics()

        # profile = config.get_stream(rs.stream.depth)
        # intrinsics = profile.as_video_stream_profile().get_intrinsics()
        return intrinsics

