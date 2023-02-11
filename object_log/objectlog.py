import armorplate
import numpy as np
class objectlog:
    

    def __init__(self, timestamp):
        self.plates = []
        self.idAssign = 0
        self.timeStamp = timestamp

    #Input from depth
    def boxesInput(self, boxList, timestamp):
        self.timeStamp = timestamp
        # add all bounding boxes to plates if plates is empty
        if len(self.plates) == 0:
            for i in boxList:
                newPlate = armorplate(i, self.idAssign)
                self.plates.append(newPlate)
                self.idAssign += 1
        else:
            for i in boxList:
                i.predictPosition(delta_t)
            # TODO: run pseudo greedy algorithm here 
                # make a new armor plate, compare it with other armor plates
            return
            
        
    #Input from prediction/errorchecking?
    def predictionInput(self, input):
        for i in input:
            self.plates[i].updateVelocity(input[i].velocity)
        return
        
    def check_plates(self, plate):
        
        
        return


