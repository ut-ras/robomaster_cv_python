from depth_calculation import depth_calculation as dp
from object_detection import object_detection as od
from communication import communication as com
import cv2
import numpy as np
import time
#begin initialization
def run_forever():
	dp.initialize_real_sense()
	detector = od.object_detector()
	detector.initialize_object_detections()
	com.initialize_communication()
	boundingbox_list = []
	while True:
		start_time = time.time()
		color_image, depth_image = dp.get_color_depth_image()
		detector.run_object_detections(color_image, boundingbox_list)
		
		if(len(boundingbox_list) == 0):
			continue

		dp.set_all_bounding_box_depth_values(depth_image, boundingbox_list)

		

		# for i in range(len(boundingbox_list)):
		# 	print(boundingbox_list[i].get_depth())
		
		boundingbox_list.clear()

		# Show images, calculate time elapsed, debugging
		cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
		cv2.imshow('RealSense', color_image)
		cv2.waitKey(1)



if __name__ == "__main__":
	run_forever()

