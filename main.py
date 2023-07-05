from depth_calculation import depth_calculation as dp
from object_detection import object_detection as od
from communication import communication as com
from pixel_to_point import pixel_to_point as ptp
from prediction import prediction
from object_log import objectlog
import logging
import sys
import cv2
import numpy as np
import time
#begin initialization
def run_forever():
	logging.basicConfig(filename='logging/log'+str(time.time())+'.log',encoding='utf-8',level=logging.DEBUG)
	RealSense = dp.RealSense()
	RealSense.initialize_real_sense()
	detector = od.object_detector()
	detector.initialize_object_detections()
	com.initialize_communication()
	boundingbox_list = []
	intrinsics = RealSense.get_intrinsics()
	object = objectlog.objectlog()

	while True:
		system_begin_time = time.time()
		is_depth_invalid = False

		start_time = time.time()
		RealSense.get_color_depth_image()
		end_time = time.time()
		logging.debug("Obtaining frame: " + str(end_time - start_time) + ' seconds')

		start_time = time.time()
		detector.run_object_detections(RealSense, boundingbox_list)
		end_time = time.time()
		logging.debug("Running detections: " + str(end_time - start_time) + ' seconds')

		if(len(boundingbox_list)==0):
			end_time = time.time()
			logging.debug('No detections, skipping frame: ' + str(end_time - system_begin_time) + ' seconds')
			com.send_no_data()
			boundingbox_list.clear()
			continue

		is_depth_invalid = RealSense.set_all_bounding_box_depth_values(boundingbox_list)

		if(is_depth_invalid):
			
			com.send_no_data()

			end_time = time.time()
			logging.debug('No valid depth data, skipping frame: ' + str(end_time - system_begin_time) + ' seconds')

			boundingbox_list.clear()
			continue

		ptp.set_point_coords(boundingbox_list,intrinsics)

		pos, vel, acc = object.select_target(boundingbox_list)

		com.send_turret_data(pos, vel, acc, hasTarget=True)
		end_time = time.time()
		logging.debug('Sending data to MCB: '+str(end_time-start_time) + ' seconds')

		start_time = time.time()
		cv2.imwrite('images/' + str(time.time()) + '.jpg',RealSense.get_color_image())
		end_time = time.time()
		logging.debug('Writing images to disk: ' + str(end_time - start_time) + ' seconds')
		logging.debug('System ran correctly: ' + str(end_time - system_begin_time) + ' seconds')

		boundingbox_list.clear()



if __name__ == "__main__":
	run_forever()

