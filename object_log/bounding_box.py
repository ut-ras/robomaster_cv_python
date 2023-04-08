import time

class BoundingBox:
	"""
	The bounding box object is an object within the armor plate object. 
	Bounding box is used to create armor plates inside object log.
	Bounding box object should be created/initialized by yolov5
	The depth value will be put into the bounding box object by depth calculation.

	Bounding box object is used in Object Log as the object to be inputed into the function.
	"""
	
	def __init__(self, x_center, y_center, width, height):
		"""
		Initialized the bounding box 
		Rundown of the values in bounding box:
		x: x position of the bounding box
		y: y position of the bounding box
		depth_value: depth position of the bounding box, based off the x, y values
		height: height position of the image
		width: width position of the image
		position: a combine vector of the x, y, depth 
		time: time value the bounding box was initialized at (in seconds)
		"""
		
		self.__x_center__ = x_center
		self.__y_center__ = y_center
		self.__depth_value__ = 0
		self.__height__ = height
		self.__width__ = width
		self.__position__ = [self.__x_center__, self.__y_center__, self.__depth_value__]
		self.__time__ = time.time()
    
	"""
	set_x_value sets the x center of the bounding box taken from the camera.
	"""
	def set_x_value(self, x: float):
		self.__x_center__ = x

	"""
	set_y_value sets the y center of the bounding box taken from the camera.
	"""
	def set_y_value(self, y: float):
		self.__y_center__ = y
	
	"""
	set_depth sets the depth at which the bounding box is located, taken from depth calculations.
	"""
	def set_depth(self, depth: float):
		self.__depth_value__ = depth
	
	"""
	set_height sets the height of the bounding box.
	"""
	def set_height(self, height: float):
		self.__height__ = height
	
	"""
	set_width sets the width of the bounding box.
	"""
	def set_width(self, width: float):
		self.__width__ = width

	"""
	set_time sets the time in seconds since epoch.
	"""
	def set_time(self):
		self.__time__ = time.time()
	
	"""
	get_x_value returns the x center of the bounding box taken from the camera.
	"""
	def get_x_value(self):
		return self.__x_center__
	
	"""
	gets _y_value returns the y center of the bounding box taken from the camera.
	"""
	def get_y_value(self):
		return self.__y_center__
	
	"""
	get_depth returns the depth of the bounding box, the depth at which the armor plate object is located real time.
	"""
	def get_depth(self):
		return self.__depth_value__
	
	"""
	get_height returns the height of the bounding box.
	"""
	def get_height(self):
		return self.__height__
	
	"""
	get_width returns the witdh of the bounding box.
	"""
	def get_width(self):
		return self.__width__
	
	"""
	get_time returns the time in seconds since epoch.
	"""
	def get_time(self):
		return self.__time__
	
	"""
	print_time prints the time in seconds since epoch.
	"""
	def print_time(self):
		print(self.__time__)

	"""
	getPosition returns the position of the armor plate in an array of [xcenter, ycenter, depth].
	"""
	def get_position(self):
		return self.__position__