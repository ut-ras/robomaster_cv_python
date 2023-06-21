import onnxruntime as ort
import numpy as np
import cv2
import time
from bounding_box import bounding_box as bb

model_path = '/home/debian/robomaster_CV/object_detection/last_with_shapes.onnx'
artifacts_dir = '/home/debian/robomaster_CV/object_detection/tidl_output'
class object_detector:

	def __init__(self):
		self.__sess__ = None
		self.__height__ = None
		self.__width__ = None
		self.__input_name__ = None

	def initialize_object_detections(self):
		session_options = ort.SessionOptions()
		
		print("Available execution providers: ", ort.get_available_providers())

		runtime_options = {
			"platform":"J7",
			"version":"8.2",
			"artifacts_folder":artifacts_dir
		}

		desired_eps = ['TIDLExecutionProvider','CPUExecutionProvider']
		self.__sess__ = ort.InferenceSession(
			model_path,
			providers = desired_eps,
			provider_options=[runtime_options,{}],
			sess_options=session_options
		)

		input_details, = self.__sess__.get_inputs()
		batch_size, channel, self.__height__, self.__width__ = input_details.shape
		print(f"Input shape: {input_details.shape}")

		assert isinstance(batch_size, str) or batch_size == 1
		assert channel == 3

		self.__input_name__ = input_details.name
		input_type = input_details.type

		print(f'Input "{self.__input_name__}": {input_type}')

	def render_boxes(self, image, output, boundingbox_list):
		CONFIDENCE_THRESHOLD = 0.2
		assert len(output.shape) == 3
		output_count = output.shape[1]

		for i in range(output_count):
			#x1, y1 coordinates for upper left corner of bounding box
			#x2, y2 coordinates for lower right corner of bounding box
			#use these to calculate height and width of bounding box, if necessary
			#use these to also determine average depth measurements of bounding box
			x1, y1, x2, y2, confidence, class_idx_float = output[0, i, :]

			if confidence < CONFIDENCE_THRESHOLD:
				continue
			
			# if part of an armor plate is detected, detection coordinates
			# are returned as a negative value. To fix, set values to 0 to avoid issues further on.
			if x1 < 0:
				x1 = 0.0
			if y1 < 0:
				y1 = 0.0
				
			# print("x1 ", x1)
			# print("y1 ", y1)
			# print("x2 ", x2)
			# print("y2 ", y2)
			# print("confidence ", confidence)
			print("class_idx_float ", class_idx_float)

			x1 = int(round(x1 / self.__width__ * image.shape[1]))
			y1 = int(round(y1 / self.__height__ * image.shape[0]))
			x2 = int(round(x2 / self.__width__ * image.shape[1]))
			y2 = int(round(y2 / self.__height__ * image.shape[0]))

			# Yes, TI outputs the class index as a float...
			#0. as red, 1. as blue
			class_draw_color = {
				# Colors for boxes of each class, in (R, G, B) order.
				0.: (255, 50, 50),
				1.: (50, 50, 255),
				# TODO: if using more than two classes, pick some more colors...
			}[class_idx_float]

			#print bounding box onto image, for debugging purposes
			cv2.rectangle(image, (x1, y1), (x2, y2), class_draw_color, 3)
			
			bounding_box = bb.BoundingBox()
			bounding_box.set_x_value(x1, x2)
			bounding_box.set_y_value(y1, y2)
			bounding_box.calculate_height()
			bounding_box.calculate_width()
			bounding_box.set_time()
			boundingbox_list.append(bounding_box)
		

	def run_object_detections(self, image, boundingbox_list):
		# YOLOv5 normalizes RGB 8-bit-depth [0, 255] into [0, 1]
		# Model trained with RGB channel order but OpenCV loads in BGR order, so reverse channels.
		input_data = cv2.resize(image, (self.__width__, self.__height__)).transpose((2, 0, 1))[::-1, :, :] / 255

		input_data = input_data.astype(np.float32)
		input_data = np.expand_dims(input_data, 0)
		
		detections, = self.__sess__.run(None, {self.__input_name__: input_data})

		#for testing purposes
		detected_target = self.render_boxes(image, detections[0, :, :, :], boundingbox_list)
		return detected_target
	
	

