The initialization was written as part of the Python RealSense C++ Library Wrapper.

We first read in the depth and color image in the loop. In the future, we will be getting rid of the color image as it is only used for the OpenCV camera feed viewer

The depth image and the color image are going to be then converted into a numpy array. If intuition serves us right, the depth image assigns a depth value for each pixel in the image. If we are fed a numpy array of the coordinates from the **yolo** model, we can use these coordinates to index into the numpy array that was converted from a depth image and feed this value into the pipeline. 

***TODO***
1. Research if the numpy array converted from the depth image actually contains correct depth values. This can be done either by researching the API or just by printing the matrix when the camera is at a fixed position and confirming the values.
2. Comment the values in the initialization for the image size due to the fact that the values coming from ***yolo*** are going to be a numpy array (this is also theoretical and will have to be addressed later) and that the numpy array coming from yolo may be of a different size as the armor plate is of a different size as well.
3. Reading in the images from the camera will have to be a continuous operation without blocking the main thread. Python offers two main methods of parallel execution, namely, multi-threading and multi-processing. 
	a. Multi-threading is more lightweight as the threads share a heap and multi-processing is more heavyweight due to the fact that each process gets a separate heap. If we lack computational power, this will be a good idea
	b. Multi-processing however, has independent I/O scheduling, allowing multiple sources of I/O without blocking which can be useful since we will have the camera running and a stream of data outputting through UART to the dev board.

Of course, this is all theoretical and will continue to be researched/tested.