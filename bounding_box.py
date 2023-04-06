import time

class BoundingBox:
	def __init__(self, x_center, y_center, width, height):
		self.__x_center__ = x_center
		self.__y_center__ = y_center
		self.__depth_value__ = 0
		self.__height__ = height
		self.__width__ = width
		self.__position__ = [self.__x_center__, self.__y_center__, self.__depth_value__]
		self.__time__ = time.time()

	def __repr__(self) -> str:
		return f"Center: ({self.__x_center__}px, {self.__y_center__}px)\n\
      	Width: {self.__width__}px\n\
        Height: {self.__height__}px\n\
        Initalized at: {self.__time__}\n"
       
	def set_x_value(self,x):
		self.__x_center__ = x

	def set_y_value(self,y):
		self.__y_center__ = y
	
	def set_depth(self, depth):
		self.__depth_value__ = depth
	
	def set_height(self, height):
		self.__height__ = height
	
	def set_width(self, width):
		self.__width__ = width

	def set_time(self):
		self.__time__ = time.time()
	
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

	def get_position(self):
		return self.__position__

	
	