import onnxruntime as ort
import numpy as np
import cv2

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

	def render_boxes(self, image, output):
		CONFIDENCE_THRESHOLD = 0
		assert len(output.shape) == 3
		output_count = output.shape[1]

		for i in range(output_count):
			x1, y1, x2, y2, confidence, class_idx_float = output[0, i, :]

			print("x1 ", x1)
			print("y1 ", y1)
			print("x2 ", x2)
			print("y2 ", y2)
			print("confidence ", confidence)
			print("class_idx_float ", class_idx_float)
			if confidence < CONFIDENCE_THRESHOLD:
				continue

			x1 = int(round(x1 / self.__width__ * image.shape[1]))
			y1 = int(round(y1 / self.__height__ * image.shape[0]))
			x2 = int(round(x2 / self.__width__ * image.shape[1]))
			y2 = int(round(y2 / self.__height__ * image.shape[0]))

			# Yes, TI outputs the class index as a float...
			class_draw_color = {
				# Colors for boxes of each class, in (R, G, B) order.
				0.: (255, 50, 50),
				1.: (50, 50, 255),
				# TODO: if using more than two classes, pick some more colors...
			}[class_idx_float]

			# Reverse RGB tuples since OpenCV images default to BGR
			print("4 ", isinstance(image,np.ndarray))
			cv2.rectangle(image, (x1, y1), (x2, y2), class_draw_color[::-1], 3)
			print("5 ", isinstance(image,np.ndarray))
			return image

	def run_object_detections(self,image):
		# YOLOv5 normalizes RGB 8-bit-depth [0, 255] into [0, 1]
		# Model trained with RGB channel order but OpenCV loads in BGR order, so reverse channels.
		input_data = cv2.resize(image, (self.__width__, self.__height__)).transpose((2, 0, 1))[::-1, :, :] / 255

		input_data = input_data.astype(np.float32)
		input_data = np.expand_dims(input_data, 0)
		
		detections, = self.__sess__.run(None, {self.__input_name__: input_data})

		#for testing purposes
		print("2 ",isinstance(image,np.ndarray))
		self.render_boxes(image, detections[0, :, :, :])
		print("3 ",isinstance(image,np.ndarray))
		return detections
	
	

