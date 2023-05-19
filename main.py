from depth_calculation import depth_calculation as dp
from object_detection import object_detection as od
import cv2
import numpy as np
#begin initialization
def run_forever():
	dp.initialize_real_sense()
	detector = od.object_detector()
	detector.initialize_object_detections()
	while True:
		color_image, depth_image = dp.get_color_depth_image()
		detector.run_object_detections(color_image)
		# Show images
		cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
		cv2.imshow('RealSense', color_image)
		cv2.waitKey(1)


if __name__ == "__main__":
	run_forever()

