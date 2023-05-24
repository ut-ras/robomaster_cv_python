from depth_calculation import depth_calculation as dp
from object_detection import object_detection as od
from communication import communication as com
import cv2
import numpy as np
#begin initialization
def run_forever():
	dp.initialize_real_sense()
	detector = od.object_detector()
	detector.initialize_object_detections()
	com.initialize_communication()
	boundingbox_list = []
	while True:
		color_image, depth_image = dp.get_color_depth_image()
		detector.run_object_detections(color_image, boundingbox_list)
		dp.set_all_bounding_box_depth_values(depth_image, boundingbox_list)

		if(boundingbox_list.empty() is not True):
			#debugging
			print("x coordinate of center of armor plate: ", boundingbox_list[:-1].get_x_center())
			print("y coordinate of center of armor plate: ", boundingbox_list[:-1].get_y_center())
			print("Depth value from last detected armor plate: ", boundingbox_list[:-1].get_depth())

			#TODO
			#Obtain velocity, acceleration values from Kalman filter
			#Research how the fuck you use the Kalman filter API
			x_center = boundingbox_list[:-1].get_x_center()
			y_center = boundingbox_list[:-1].get_y_center()
			depth_value = boundingbox_list[:-1].get_depth()

			#debugging
			assert(isinstance(x_center,np.float32))
			assert(isinstance(y_center,np.float32))
			assert(isinstance(depth_value,np.float32))

			#send message to dev board through UART
			# com.send_turret_data(x_center,y_center,depth_value,np.float32(0),np.float32(0),np.float32(0),np.float32(0),np.float32(0),np.float32(0),True)
			boundingbox_list.clear()



		# Show images, debugging
		cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
		cv2.imshow('RealSense', color_image)
		cv2.waitKey(1)


if __name__ == "__main__":
	run_forever()

