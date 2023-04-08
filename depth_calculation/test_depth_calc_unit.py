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

#test to make sure that we are receiving a depth value, using the bounding box data from the realsense
#that we can run predictions on
def test_depth_value_at_bounding_box():
	
	depth_frame = dc.get_depth_value_from_bounding_box()
	assert isinstance(depth_frame,np.ndarray) is True
	assert depth_frame.shape == (720,1280)


