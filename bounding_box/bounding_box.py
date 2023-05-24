import time
import numpy as np
class BoundingBox:
	def __init__(self):
		self.__x1__ = 0
		self.__x2__ = 0
		self.__y1__ = 0
		self.__y2__ = 0
		self.__x_center__ = 0
		self.__y_center__ = 0
		self.__depth_value__ = 0
		self.__height__ = 0
		self.__width__ = 0
		self.__time__ = time.local_time(time.time())

	def set_x_value(self,x1, x2):
		self.__x1__ = x1
		self.__x2__ = x2

	def set_y_value(self,y1, y2):
		self.__y1__ = y1
		self.__y2__ = y2

	def calculate_x_center(self):
		self.__x_center__ = np.float32((self.__x1__ + self.__x2__)/2)
	
	def calculate_y_center(self):
		self.__y_center__ = np.float32((self.__y1__ + self.__y2__)/2)
	
	def set_depth(self, depth):
		self.__depth_value__ = depth
	
	def calculate_height(self):
		self.__height__ = self.__y2__ - self.__y1__
	
	def calculate_width(self):
		self.__width__ = self.__x2__ - self.__x1__

	def set_time(self):
		self.__time__ = time.localtime(time.time())
	
	def get_x_value(self):
		return self.__x1__, self.__x2__
	
	def get_y_value(self):
		return self.__y1__, self.__y2__
	
	def get_x_center(self):
		self.calculate_x_center()
		return self.__x_center__
	
	def get_y_center(self):
		self.calculate_y_center()
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
	
	