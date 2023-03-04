import time

class BoundingBox:
	def __init__(self):
		self.__x_center__ = 0
		self.__y_center__ = 0
		self.__depth_value__ = 0
		self.__height__ = 0
		self.__width__ = 0
		self.__time__ = time.local_time(time.time())

	def set_x_value(self,x):
		self.__x_center__ = x

	def set_y_value(self,y):
		self.__y_center__ = y
	
	def set_depth(self, depth):
		assert(depth != 0)
		assert(depth < 12)
		self.__depth_value__ = depth
	
	def set_height(self, height):
		self.__height__ = height
	
	def set_width(self, width):
		self.__width__ = width

	def set_time(self):
		self.__time = time.local_time(time.time())
	
	def get_x_value(self):
		return self.__x_center__
	
	def get_y_value(self):
		return self.__y_center__
	
	def get_depth(self):
		return self.__depth_value__
	
	def get_height(self):
		return self.__height__
	
	def get_width(self):
		return self.__width__
	
	def get_time(self):
		return self.__time__
	
	def print_time(self):
		print(self.__time__)
	
	