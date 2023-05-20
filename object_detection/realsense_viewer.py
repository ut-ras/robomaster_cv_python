import torch
import pyrealsense2 as rs
import cv2
import sys
import numpy as np

#Configure color stream
pipeline = rs.pipeline()
def initialize_realsense():
        
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

	if device_product_line == 'L500':
		config.enable_stream(rs.stream.color, 960, 540, rs.format.bgr8, 30)
	else:
		config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

	# Start streaming
	pipeline.start(config)

def initialize_yolo():
	model = torch.hub.load('yolov5','custom',path='last.pt',source='local')
	return model

def run_detections_on_camera(model):
	try:
		while True:

			# Wait for a coherent pair of frames: depth and color
			frames = pipeline.wait_for_frames()
			color_frame = frames.get_color_frame()
			if not color_frame:
				continue

			# Convert images to numpy arrays

			color_image = np.asanyarray(color_frame.get_data())
			prediction = model(color_image, size=480)
			print(prediction)

			# Show images
			cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
			cv2.imshow('RealSense', color_image)
			cv2.waitKey(1)

	finally:

		# Stop streaming
		pipeline.stop()

if __name__ == '__main__':
	initialize_realsense()

	model = initialize_yolo()
	run_detections_on_camera(model)