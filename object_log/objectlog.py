import armorplate
import numpy as np
class objectlog:
    

    def __init__(self, timestamp):
        self.plates = []
        self.idAssign = 0
        self.timeStamp = timestamp
        # TODO: instantiate file to write to
        self.objectLogOuput = open("ObjectLog.txt",'w')

    #Input from depth
    def boxesInput(self, boxList, timestamp, currentTime):
        self.timeStamp = timestamp
        # add all bounding boxes to plates if plates is empty
        if len(self.plates) == 0:
            for i in boxList:
                newPlate = armorplate(i, self.idAssign)
                self.plates.append(newPlate)
                self.idAssign += 1
        else:
            for i in boxList:
                i.predictPosition(currentTime)
            # TODO: run pseudo greedy algorithm here 
                # make a new armor plate, compare it with other armor plates
            return
            
        
    #Input from prediction/errorchecking?
    def predictionInput(self, input):
        for i in input:
            self.plates[i].updateVelocity(input[i].velocity)
        return
        
    #unsure
    def check_plates(self, plate):
        
        
        return

    # pseudo greedy (?)
    #newPlates would the the new armor plates that we are receiving
    def assign_plates(self, newPlates): 
        for i in newPlates:
            
        return
    
    #log all of the armor plates at the very end of the program before all the plates are deleted (genocide D:)
    def kill_plates(self):
        for plate in self.plates:
            plate.writeToHistory(self.objectLogOutput)
            self.kill_plate(self.plates.index(plate))
        self.objectLogOutput.close()
        return

    #logs and removes a single plate
    def kill_plate(self, index):
        self.plates[index].writeToHistory(self.objectLogOuput)
        self.plates.remove(index)
