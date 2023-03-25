import numpy as np
import bounding_box as bb

class armorplate:
    """
    A object of an armor plate object that is typically associated with what is seen.

    Object should be initialized from the outputs of the machine learning model and has parameters such as position, velocity, depth, etc.
    
    Armor plate object is used in object log
    """

    def __init__(self, boundingbox, id: int):
        """
        Initializes the armor plate
        Rundown of the fields of the object:
        position, holds the position of the bounding box in a x, y, z system (camera relative)
        velocity, last velocity of the target
        acceleration, last acceleration of the target
        boundingbox, ???
        id, id of the armor plate (fokr debugging purposes)
        activity, boolean on the plate on if it is currently active
        
        """
        self.position = boundingbox.position
        self.velocity = 0 
        self.acceleration = 0
        self.boundingbox = boundingbox # bounding box object
        self.id = id
        self.activity = True
        self.timeBuffer = 0
        self.nextPosition = [0,0,0]
        self.lastTime = 0
        self.history = None
        self.timestamp_history = None # Hasif told me to make this
        self.assoc_plates = None 
        self.max_assoc_plates = 5

    def updateVA(self, velocity: float, acceleration: float) -> int:
        self.velocity = velocity
        self.acceleration = acceleration
        return 0

    # use position, velocity, and delta time to predict where armor plate will be
    # this method is subject to change depending on how PVA is implemented (currently assumed to be world positions)
    def predictPosition(self, currentTime):
        delta_t = currentTime - lastTime
        delta_t = delta_t / 1000
        self.nextPosition = self.position + (self.velocity * delta_t) + (self.acceleration * np.exp(delta_t, 2) / 2) # kinematics :D
        lastTime = currentTime

    def getNextPosition(self):
        return self.nextPosition

    def getBoundingBox(self) -> bb.BoundingBox:
        return self.boundingbox

    def getLastTime(self):
        self.lastTime
    
    # given an armorplate we have associated in objectlog, 
    # add to a list of associated armor plates
    def addArmorPlate(self, new_plate):
        #add to data structure
        if len(self.assoc_plates) == self.max_assoc_plates:
            self.assoc_plates.pop()
        self.assoc_plates.insert(0,new_plate)
        return



    def writeToHistory(self, historyFile):
        # historyFile = open("pathhere",'a')
        historyFile.write(self.id + " " + self.position + " " self.activity + " " )
        historyFile.write(self.lastTime + ", ") # insert writing stuff here
        # historyFile.close()