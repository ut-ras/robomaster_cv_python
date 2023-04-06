import numpy as np
import bounding_box as bb
import prediction as pred


class ArmorPlate:
    """
    A object of an armor plate object that is typically associated with what is seen.

    Object should be initialized from the outputs of the machine learning model and has parameters such as position, velocity, depth, etc.
    
    Armor plate object is used in object log
    """

    def __init__(self, boundingbox:bb, id: int):
        """
        Initializes the armor plate
        Rundown of the fields of the object:
        position, holds the position of the bounding box in a x, y, z system (camera relative)
        velocity, last velocity of the target
        acceleration, last acceleration of the target
        boundingbox, boundingbox object
        id, id of the armor plate (for debugging purposes)
        activity, boolean on the plate on if it is currently active
        
        """
        self.position = boundingbox.get_position()
        self.velocity = [0,0,0]
        self.acceleration = [0,0,0]
        self.boundingbox = boundingbox # bounding box object
        self.id = id
        self.activity = True
        self.timeBuffer = 0
        self.nextPosition = [0,0,0]
        self.lastTime = None
        self.history = None
        self.timestamp_history = None # Hasif told me to make this
        self.assoc_plates = None 
        self.max_assoc_plates = 5
        self.kf = pred.Prediction()

    # velocity and acceleration are sets of three values
    def updateVA(self):
        va_vector = self.kf.getVA()
        self.velocity = va_vector[0:3]
        self.acceleration = va_vector[3:6]

    # use position, velocity, and delta time to predict where armor plate have moved to since the last check
    # this method is subject to change depending on how PVA is implemented (currently assumed to be world positions)
    # assumes that time is being kept track of in per second units while velocity/acceleration are per millisecond units
    def predictPosition(self, currentTime):
        delta_t = currentTime - self.lastTime
        self.nextPosition = self.position + (np.multiply(self.velocity, delta_t)) + (np.multiply(self.acceleration, np.exp(delta_t, 2) / 2)) # kinematics :D
        
    def getPosition(self):
        return self.position 

    def getID(self):
        return self.id   

    def getNextPosition(self):
        return self.nextPosition

    def getBoundingBox(self) -> bb.BoundingBox:
        return self.boundingbox

    def getLastTime(self):
        self.lastTime

    def setActivity(self, newAct):
        self.activity = newAct
    
    # given an armorplate we have associated in objectlog, 
    # add to a list of associated armor plates
    def addArmorPlate(self, new_plate, currentTime):
        #add to data structure
        if len(self.assoc_plates) == self.max_assoc_plates:
            self.assoc_plates.pop()
        self.assoc_plates.insert(0,new_plate)
        self.kf.kinematicUpdate(new_plate.getPosition())
        self.lastTime = currentTime

        return

    def writeToHistory(self, historyFile):
        # historyFile = open("pathhere",'a')
        historyFile.write(self.id + " " + self.position + " " + self.activity + " " )
        historyFile.write(self.lastTime + ", ") # insert writing stuff here
        # historyFile.close()