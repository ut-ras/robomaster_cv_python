import depth_calculation as dp
import object_detection as od
import cv2

#begin initialization
detector = None
def initialization():
	dp.initialize_real_sense()
	detector = od.object_detector()
	detector.initialize_object_detections()

def run_forever():
	while True:
		color_image, depth_image = dp.get_color_depth_image()
		color_image = detector.run_object_detections(color_image)
				# Show images
		cv2.namedWindow('RealSense', cv2.WINDOW_AUTOSIZE)
		cv2.imshow('RealSense', color_image)
		cv2.waitKey(1)


if __name__ == "__main__":
	initialization()
	run_forever()

