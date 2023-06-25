from depth_calculation import depth_calculation as dp
from object_detection import object_detection as od
from communication import communication as com
from pixel_to_point import pixel_to_point as ptp
from prediction import prediction
from object_log import objectlog
import cv2
import numpy as np
import time
#begin initialization
def run_forever():
	RealSense = dp.RealSense()
	RealSense.initialize_real_sense()
	detector = od.object_detector()
	detector.initialize_object_detections()
	com.initialize_communication()
	boundingbox_list = []
	intrinsics = RealSense.get_intrinsics()
	object = objectlog.objectlog()
	no_data_pos = {
		'x_pos':np.float32(0),
		'y_pos':np.float32(0),
		'z_pos':np.float32(0)
	}
	no_data_vel = {
		'x_vel':np.float32(0),
		'y_vel':np.float32(0),
		'z_vel':np.float32(0)
	}
	no_data_acc = {
		'x_acc':np.float32(0),
		'y_acc':np.float32(0),
		'z_acc':np.float32(0)
	}

	while True:
		is_depth_invalid = False

		RealSense.get_color_depth_image()

		detector.run_object_detections(RealSense, boundingbox_list)

		if(len(boundingbox_list)==0):
			
			com.send_no_data()
			boundingbox_list.clear()
			continue

		is_depth_invalid = RealSense.set_all_bounding_box_depth_values(boundingbox_list)

		if(is_depth_invalid):

			com.send_no_data()
			boundingbox_list.clear()
			continue

		ptp.set_point_coords(boundingbox_list,intrinsics)

		pos, vel, acc = object.select_target(boundingbox_list)

		com.send_turret_data(pos, vel, acc, hasTarget=True)
		
		boundingbox_list.clear()



if __name__ == "__main__":
	run_forever()

