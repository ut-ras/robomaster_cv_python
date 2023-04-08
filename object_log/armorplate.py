if __name__ == "__main__":
    import bounding_box as bb
    import prediction as pred
    
import numpy as np
from prediction import prediction as pred
from object_log import bounding_box as bb


class ArmorPlate:
    MAX_ASSOC_PLATES = 5
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
        self.velocity = [0,0,0] # [xvelocity, yvelocity, zvelocity]
        self.acceleration = [0,0,0] # [xacceleration, yacceleration, zacceleration]
        self.boundingbox = boundingbox # bounding box object
        self.id = id
        self.activity = True
        self.timeBuffer = 0
        self.nextPosition = [0,0,0]
        self.lastTime = None
        self.assoc_plates = [] # used for history
        self.kf = pred.Prediction()

    # Velocity and acceleration are sets of three values.
    """
    Input from Armorplate
    Input:
        - Current Velocity vector of armor plate
        - Current Acceration vector of armor plate
    Output:
        - Predicated Velocity vector of armor plate
        - Predicted Acceration vector of armor plate
    """
    def updateVA(self):
        va_vector = self.kf.getVA()
        self.velocity = va_vector[0:3]
        self.acceleration = va_vector[3:6]

    """
    predictPosition uses position, velocity, and delta time to predict where armor plate have moved to since the last check;
    this method is subject to change depending on how PVA is implemented (currently assumed to be world positions)
    assumes that time is being kept track of in per second units while velocity/acceleration are per millisecond units.
    """
    def predictPosition(self, currentTime: float):
        delta_t = currentTime - self.lastTime
        self.nextPosition = self.position + (np.multiply(self.velocity, delta_t)) + (np.multiply(self.acceleration, np.exp(delta_t, 2) / 2)) # kinematics :D
    
    """
    getPosition returns the position of the armor plate in an array of [xcenter, ycenter, depth].
    """
    def getPosition(self):
        return self.position 

    """
    getID returns the id of the armor plate.
    """
    def getID(self):
        return self.id   

    """"
    getNextPosition returns the predicted next position of the armor plate in an array of 
    [xcenter, ycenter, depth].
    """
    def getNextPosition(self):
        return self.nextPosition

    """"
    getBoundingBox returns the bounding box object.
    """
    def getBoundingBox(self) -> bb.BoundingBox:
        return self.boundingbox
    
    """
    getLastTime returns the time the armor plate object was last seen.
    """
    def getLastTime(self):
        return self.lastTime

    """
    setActivity sets if the armor plate is currently active.
    """
    def setActivity(self, newAct: bool):
        self.activity = newAct
    
    """
    Given an armorplate we have associated in objectlog, 
    add to a list of associated armor plates.
    """
    def addArmorPlate(self, new_plate: bb, currentTime: float):
        #add to data structure
        if len(self.assoc_plates) == self.max_assoc_plates:
            self.assoc_plates.pop()
        self.assoc_plates.insert(0,new_plate)
        self.kf.kinematicUpdate(new_plate.getPosition())
        self.lastTime = currentTime

        return

    """
    writeToHistory writes the ID, position, activity, and lastTime variables to a text file in a
    formatted manner.
    """
    def writeToHistory(self, historyFile):
        # historyFile = open("pathhere",'a')
        historyFile.write("ID: {} Position: {} Activity: {} LastTime: {}\n"\
                          .format(self.id, self.position, self.activity, self.lastTime))
        # historyFile.close()