import numpy as np
from object_log.bounding_box import BoundingBox as bb
import object_log.objectlog as objl

# Test code to test out object log

'''
def __init__(self, x_center, y_center, width, height):
		self.__x_center__ = x_center
		self.__y_center__ = y_center
		self.__depth_value__ = 0
		self.__height__ = height
		self.__width__ = width
		self.__position__ = [self.__x_center__, self.__y_center__,self.__depth_value__]
		self.__time__ = time.time()
'''

objectLog = objl.objectlog()

# 720 x 1280 bounds.
# Operating distance of 3-5 meters.

# First set of inputs.
inputSet1 = []
inputSet1.append(bb.BoundingBox(100, 100, 10, 10)) #0
inputSet1[0].set_depth(1.3)
inputSet1[0].set_time(0.0)
inputSet1.append(bb.BoundingBox(200, 200, 10, 10)) #1
inputSet1[1].set_depth(1.8)
inputSet1[1].set_time(0.0)
inputSet1.append(bb.BoundingBox(300, 300, 10, 10)) # 2
inputSet1[2].set_depth(1)
inputSet1[2].set_time(0.0)
inputSet1.append(bb.BoundingBox(400, 400, 10, 10)) # 3
inputSet1[3].set_depth(1.9)
inputSet1[3].set_time(0.0)
inputSet1.append(bb.BoundingBox(650, 670, 10, 10)) # 4
inputSet1[4].set_depth(0.5)
inputSet1[4].set_time(0.0)
inputSet1.append(bb.BoundingBox(100, 600, 10, 10)) # 5
inputSet1[5].set_depth(1.75)
inputSet1[5].set_time(0.0)
inputSet1.append(bb.BoundingBox(700, 550, 10, 10)) # 6
inputSet1[6].set_depth(1.2)
inputSet1[6].set_time(0.0)
objectLog.boxesInput(inputSet1, 0)

# Note: a decrease in y means that its moving up, a increase in y means its moving down.
# This is because the camera input is top left corner relative.

'''
0 - move right and down - depth should decrease
1 - move left - depth should stay the same
2 - moving left and down - depth should decrease
3 - moving right and up - depth should increase
4 - no movement - depth should stay same
5 - moving down and right - depth should decrease
6 - moving right and up - depth should increase
'''
time = 0.033
for i in range(10):
	newInput = []

	#0
	newInput.append(bb.BoundingBox(inputSet1[0].get_x_value() + 20, inputSet1[0].get_y_value() + 20), 10, 10)
	newInput[0].set_depth(inputSet1[0].get_depth() - .1)
	#1
	newInput.append(bb.BoundingBox(inputSet1[1].get_x_value() - 20, inputSet1[1].get_y_value()), 10, 10)
	newInput[0].set_depth(inputSet1[1].get_depth())
	#2
	newInput.append(bb.BoundingBox(inputSet1[2].get_x_value() - 20, inputSet1[2].get_y_value() + 20), 10, 10)
	newInput[0].set_depth(inputSet1[2].get_depth() - .1)
	#3
	newInput.append(bb.BoundingBox(inputSet1[3].get_x_value() + 20, inputSet1[3].get_y_value() - 20), 10, 10)
	newInput[0].set_depth(inputSet1[3].get_depth() + .1)
	#4
	newInput.append(bb.BoundingBox(inputSet1[4].get_x_value(), inputSet1[4].get_y_value()), 10, 10)
	newInput[0].set_depth(inputSet1[4].get_depth())
	#5
	newInput.append(bb.BoundingBox(inputSet1[5].get_x_value() + 20, inputSet1[5].get_y_value() + 20), 10, 10)
	newInput[0].set_depth(inputSet1[5].get_depth() - .1)
	#6
	newInput.append(bb.BoundingBox(inputSet1[6].get_x_value() + 20, inputSet1[6].get_y_value() - 20), 10, 10)
	newInput[0].set_depth(inputSet1[6].get_depth()  +.1)
	
	#TODO make loop to set time of all elements in new input before sending to object log

# Second set of inputs.

'''
inputSet2 = []
inputSet2.append(bb.BoundingBox(150, 100, 10, 10)) #0 - moving right
inputSet2[0].set_depth(1.3)
inputSet2[0].set_time(0.033)
inputSet2.append(bb.BoundingBox(200, 500, 10, 10)) #1 - moving down
inputSet2[1].set_depth(1.3)
inputSet2[1].set_time(0.033)
inputSet2.append(bb.BoundingBox(400, 400, 10, 10)) # 2 - moving down and right
inputSet2[2].set_depth(1.25)
inputSet2[2].set_time(0.033)
inputSet2.append(bb.BoundingBox(300, 300, 10, 10)) # 3 - moving left and up
inputSet2[3].set_depth(1.4)
inputSet2[3].set_time(0.033)
inputSet2.append(bb.BoundingBox(650, 670, 10, 10)) # 4 - no movement
inputSet2[4].set_depth(1.0)
inputSet2[4].set_time(0.033)
inputSet2.append(bb.BoundingBox(200, 700, 10, 10)) # 5 - moving down and right
inputSet2[5].set_depth(1.75)
inputSet2[5].set_time(0.033)
inputSet2.append(bb.BoundingBox(600, 450, 10, 10)) # 6 - moving right and up
inputSet2[6].set_depth(1.0)
inputSet2[6].set_time(0.033)
objectLog.boxesInput(inputSet2, 0.033)
'''