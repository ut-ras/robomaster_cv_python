import onnxruntime as ort

model_path = 'last_with_shapes.onnx'
artifacts_dir = 'tidl_output'
sess = None
def initialize_object_detections():
	session_options = ort.SessionOptions()
	
	print("Available execution providers: ", ort.get_available_providers())

	runtime_options = {
		"platform":"J7",
		"version":"8.2",
		"artifacts_folder":artifacts_dir
	}

	desired_eps = ['TIDLExecutionProvider','CPUExecutionProvider']
	sess = ort.InferenceSession(
		model_path,
		providers = desired_eps,
		provider_options=[runtime_options,{}],
		sess_options=session_options
	)

	input_details, = sess.get_inputs()
	batch_size, channel, height, width = input_details.shape
	print(f"Input shape: {input_details.shape}")

	assert isinstance(batch_size, str) or batch_size == 1
	assert channel == 3

	input_name = input_details.name
	input_type = input_details.type

	print(f'Input "{input_name}": {input_type}')

#def run_object_detections(image, )




