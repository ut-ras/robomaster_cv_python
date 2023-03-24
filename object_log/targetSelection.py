import objectlog
import numpy as np

#Assumes x, y as camera 
def selectTarget(self, log, mousePos):
    shortest_dist = float('inf')
    shortest_plate = -1 # keeping the index, not decided what to do with the closest plate
    for i in range(len(log.plates)):
        if (np.pow((mousePos[0]-log.plates[i].boundingBox.__x_center__), 2)\
        +np.pow((mousePos[1]-log.plates[i].boundingBox.__y_center__), 2) < shortest_dist):
            shortest_plate = log.plates[i].id
    return shortest_plate