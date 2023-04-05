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
        shortest_plate = -1 # keeping the index, not decided what to do with the closest plate
        #Loops through all armor plates and finds the closest plate by pythagorean theorem
        for i in range(len(log.plates)):
            if (np.pow((mousePos[0]-log.plates[i].boundingBox.__x_center__), 2)\
            +np.pow((mousePos[1]-log.plates[i].boundingBox.__y_center__), 2) < shortest_dist):
                shortest_plate = log.plates[i].id
        return shortest_plate