import numpy as np

class armorplate:

    def __init__(self, boundingbox, id):
        self.position = boundingbox.position
        self.velocity = 0
        self.acceleration = 0
        self.boundingbox = boundingbox
        self.id = id
        self.activity = True
        self.timeout = 0
        self.nextPosition = [0,0,0]

    def updateVA(self, velocity, acceleration):
        self.velocity = velocity
        self.acceleration = acceleration

    # use position, velocity, and delta time to predict where armor plate will be
    # this method is subject to change depending on how PVA is implemented
    def predictPosition(self, delta_t):
        self.nextPosition = self.position + (self.velocity * delta_t) + (self.acceleration * np.exp(delta_t, 2) / 2) # kinematics :D