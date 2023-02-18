The initialization was written as part of the Python RealSense C++ Library Wrapper.

We first read in the depth image in the while loop. This is a depth frame that contains the depth values at each pixel in the image.

The depth image is going to be then converted into a numpy array. This will allow easy access to the depth values by indexing with relevant x and y values for the armor plate. This can then be returned to compute turret movement for ballistics or prediction.

***TODO***
1. Reading in the images from the camera will have to be a continuous operation without blocking the main thread. Python offers two main methods of parallel execution, namely, multi-threading and multi-processing. 
	a. Multi-threading is more lightweight as the threads share a heap and multi-processing is more heavyweight due to the fact that each process gets a separate heap. If we lack computational power, this will be a good idea
	b. Multi-processing however, has independent I/O scheduling, allowing multiple sources of I/O without blocking which can be useful since we will have the camera running and a stream of data outputting through UART to the dev board.
2. Write unit tests for the depth sensor -> think of possible scenarios that can cause the camera to fail and how we can overcome these issues.

Of course, this is all theoretical and will continue to be researched/tested.