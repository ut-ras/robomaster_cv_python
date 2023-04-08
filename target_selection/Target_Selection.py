import numpy as np


screenWidth = 1280
screenHeight = 720
mousePos = [screenWidth/2, screenHeight/2]
#Assumes x, y as camera 
class targetSelection(object):
    #Takes in an ObjectLog Object
    #Returns the id of the Plate with the shortest distance
    def selectTarget(self, log):
        shortest_dist = float('inf')
        shortest_plate = None # keeping the index, not decided what to do with the closest plate
        #Loops through all armor plates and finds the closest plate by pythagorean theorem
        for i in range(len(log.plates)):
            if (pow((mousePos[0]-log.plates[i].boundingbox.__x_center__), 2)\
            +pow((mousePos[1]-log.plates[i].boundingbox.__y_center__), 2) < shortest_dist):
                shortest_plate = log.plates[i]
                shortest_dist = pow((mousePos[0]-log.plates[i].boundingbox.__x_center__), 2)+pow((mousePos[1]-log.plates[i].boundingbox.__y_center__), 2)
        return shortest_plate