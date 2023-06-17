from depth_calculation import depth_calculation as dp
from object_detection import object_detection as od
from communication import communication as com
from pixel_to_point import pixel_to_point as ptp
import cv2
import numpy as np
import time

#for saving images
import os



#begin initialization
def run_forever():
	dp.initialize_real_sense()
	detector = od.object_detector()
	detector.initialize_object_detections()
	com.initialize_communication()
	boundingbox_list = []
	intrinsics = dp.get_intrinsics()

	#Index number for frames for file saving
	frameIndex = 0
	
	#previous image variable (to ensure that we do not record duplicate frames)
	prevImage = None

	while True:
		# start_time = time.time()
		color_image, depth_image = dp.get_color_depth_image()
		#detector.run_object_detections(color_image, boundingbox_list)

		if prevImage is None or prevImage != color_image:	#Checks to see if there is a difference between the frames
			try:
				#save frames into file
				frameIndex += 1
				file_path = ""  #insert file path here if desired
				file_name = "video"


				cv2.imwrite(file_name + str(frameIndex) + ".png",color_image)
				#below version includes saving it to a specific place in the file path
				# cv2.imwrite(os.path.join(file_path,file_name + str(frameIndex)+ ".png"),color_image)

			except:	#In case of running out of memory
				print("Ran out of mem")
		prevImage = color_image
		
		# Commented out for sole purpose is to collect and save frame

		# if(len(boundingbox_list)==0):
		# 	com.send_turret_data(xPos = np.float32(0.0), yPos = np.float32(0.0), zPos = np.float32(0.0)
		# 	,xVel = np.float32(0.0),yVel = np.float32(0.0), zVel = np.float32(0.0),
		# 	xAcc = np.float32(0.0), yAcc = np.float32(0.0), zAcc = np.float32(0.0),
		# 	hasTarget=False)
		# 	boundingbox_list.clear()
		# 	continue

		# dp.set_all_bounding_box_depth_values(depth_image, boundingbox_list)

		# ptp.set_point_coords(boundingbox_list,intrinsics)

		# com.send_turret_data(xPos = boundingbox_list[0].get_x_coord(), yPos = boundingbox_list[0].get_y_coord(), zPos = boundingbox_list[0].get_z_coord(),
		# 	xVel = np.float32(0.0),yVel = np.float32(0.0), zVel = np.float32(0.0),
		# 	xAcc = np.float32(0.0), yAcc = np.float32(0.0), zAcc = np.float32(0.0),
		# 	hasTarget=True)
		
		# boundingbox_list.clear()

		# Show images, calculate time elapsed, debugging
		# cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
		# cv2.imshow('RealSense', color_image)
		# cv2.waitKey(1)



if __name__ == "__main__":
	run_forever()

