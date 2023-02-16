import numpy as np

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
        self.boundingbox = boundingbox
        self.id = id
        self.activity = True
        self.timeout = 0
        self.nextPosition = [0,0,0]

    def updateVA(self, velocity: float, acceleration: float) -> int:
        self.velocity = velocity
        self.acceleration = acceleration
        return 0;

    # use position, velocity, and delta time to predict where armor plate will be
    # this method is subject to change depending on how PVA is implemented
    def predictPosition(self, delta_t):
        self.nextPosition = self.position + (self.velocity * delta_t) + (self.acceleration * np.exp(delta_t, 2) / 2) # kinematics :D