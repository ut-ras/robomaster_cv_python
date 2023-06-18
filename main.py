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
	# pred = prediction.Prediction()
	# pred.kinematicPredict(1.0)

	while True:
		# start_time = time.time()

		color_image, depth_image = dp.get_color_depth_image()
		detector.run_object_detections(color_image, boundingbox_list)
		armor_plate_list.extend(boundingbox_list)
		if(len(boundingbox_list)==0):
			com.send_turret_data(xPos = np.float32(0.0), yPos = np.float32(0.0), zPos = np.float32(0.0)
			,xVel = np.float32(0.0),yVel = np.float32(0.0), zVel = np.float32(0.0),
			xAcc = np.float32(0.0), yAcc = np.float32(0.0), zAcc = np.float32(0.0),
			hasTarget=False)
			boundingbox_list.clear()
			continue

		dp.set_all_bounding_box_depth_values(depth_image, boundingbox_list)

		ptp.set_point_coords(boundingbox_list,intrinsics)

		object.assoc_boxes(boundingbox_list)
		pos, vel, acc = object.select_target()



		# pos = np.ndarray([boundingbox_list[-1].get_x_coord(),boundingbox_list[-1].get_y_coord(),boundingbox_list[-1].get_z_coord()])
		# pred.kinematicUpdate(pos)
		# del_t = armor_plate_list[-1].get_time() - armor_plate_list[-2].get_time()
		# pred.kinematicPredict(del_t)
		# pos = pred.getPredictedPos()
		# vel_acc = pred.getVA()
		
		# assert isinstance(pos["x_pos"],np.float32) is True & isinstance(pos["y_pos"],np.float32) is True & isinstance(pos["z_pos"],np.float32) is True
		# assert isinstance(vel_acc["x_vel"],np.float32) is True & isinstance(vel_acc["y_vel"],np.float32) is True & isinstance(vel_acc["z_vel"],np.float32) is True
		# assert isinstance(vel_acc["x_acc"],np.float32) is True & isinstance(vel_acc["y_acc"],np.float32) is True & isinstance(vel_acc["z_acc"],np.float32) is True

		com.send_turret_data(xPos = pos["x_pos"], yPos = pos["y_pos"], zPos = pos["z_pos"],
			xVel = vel["x_vel"],yVel = vel["y_vel"], zVel = vel["z_vel"],
			xAcc = acc["x_acc"], yAcc = acc["y_acc"], zAcc = acc["z_acc"],
			hasTarget=True)
		
		boundingbox_list.clear()

		# Show images, calculate time elapsed, debugging
		# cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
		# cv2.imshow('RealSense', color_image)
		# cv2.waitKey(1)



if __name__ == "__main__":
	run_forever()

