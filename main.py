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
	dp.initialize_real_sense()
	detector = od.object_detector()
	detector.initialize_object_detections()
	com.initialize_communication()
	#list of all bounding boxes ever
	armor_plate_list = []
	boundingbox_list = []
	intrinsics = dp.get_intrinsics()
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
		start_time = time.time()
		color_image, depth_image = dp.get_color_depth_image()
		detector.run_object_detections(color_image, boundingbox_list)
		armor_plate_list.extend(boundingbox_list)
		if(len(boundingbox_list)==0):
			com.send_turret_data(no_data_pos, no_data_vel, no_data_acc, hasTarget=False)
			boundingbox_list.clear()
			continue

		dp.set_all_bounding_box_depth_values(depth_image, boundingbox_list)

		ptp.set_point_coords(boundingbox_list,intrinsics)

		object.assoc_boxes(boundingbox_list)
		pos, vel, acc = object.select_target()
		end_time = time.time()
		print("Position values",pos)
		print("Velocity values", vel)
		print("Acceleration", acc)
		print("total time for system to run", end_time - start_time)
		com.send_turret_data(pos, vel, acc, hasTarget=True)
		
		boundingbox_list.clear()

		# Show images, calculate time elapsed, debugging
		# cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
		# cv2.imshow('RealSense', color_image)
		# cv2.waitKey(1)



if __name__ == "__main__":
	run_forever()

