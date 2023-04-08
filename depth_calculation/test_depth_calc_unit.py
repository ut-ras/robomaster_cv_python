import pytest as pt
import depth_calculation as dc
import numpy as np
import pyrealsense2 as rs

#initialize realsense drivers and test to make sure that a camera is connected at the time of test running
def test_realsense_connection():
	dc.initialize_real_sense()
	config = rs.config()

	# Get device product line for setting a supporting resolution

	pipeline_wrapper = rs.pipeline_wrapper(dc.pipeline)
	pipeline_profile = config.resolve(pipeline_wrapper)
	device = pipeline_profile.get_device()
	device_product_line = str(device.get_info(rs.camera_info.product_line))

	found_rgb = False
	for s in device.sensors:
		if s.get_info(rs.camera_info.name) == 'RGB Camera':
			found_rgb = True
			break
	assert found_rgb is True

#test to make sure that we are receiving a color image from the realsense that yolo can run on
def test_color_image():
	color_image = dc.get_color_image()
	print(type(color_image))
	assert type(color_image) is rs.video_frame

# test to make sure that we are receiving a depth image of type numpya array and we do not have empty depth measurements
def test_depth_image():
	depth_image = dc.get_depth_at_pixel(0)
	assert(np.all(depth_image==0) is not True)
	assert isinstance(depth_image,np.ndarray) is True
	
	#uncomment this when depth_image function is fully written to provide the depth value on the center of the armor plate
	# instead of returning the entire np array of depth values
	# assert depth_image != 0
	# assert type(depth_image) is float 

