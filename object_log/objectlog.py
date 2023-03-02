import armorplate
import numpy as np
import bounding_box

class objectlog:
    

    def __init__(self, timestamp):
        self.plates = []
        self.idAssign = 0
        self.timeStamp = timestamp
        
        # TODO: instantiate file to write to
        self.objectLogOuput = open("ObjectLog.txt",'w')


    #Input from depth
    #Input is a bunch of bounding boxes, we need to associate each one with a previous bounding box
    # the plates unless the closest distance is greater than some margin of error
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
                new_armor = armorplate(boxList[i], self.idAssign)

                # greedy stuff done here \/
                assoc = self.assign_plate(new_armor, self.plates) #index of matching plate
                if assoc_plate == -1:
                    #new plate, not seen before
                    if len(self.plates) < 9:
                        # add new plate and we have space
                        self.plates.append(new_armor)
                    else:
                        #idk, no space to add plate
                        print("need space?")

                elif assoc_plate == -2:
                    #panic, something is null
                    print("panic")
                    
                else:
                    # associate new plate with plates[assoc]
                    # add new plate to list of associated plates of plates[assoc]
                    assoc_plate = self.plates[assoc]
                    assoc_plate.addArmorPlate(new_armor)
                    assoc_plate.timeBuffer = 0

            # bump up timer buffers and remove dead plates 
            kill_threshold = -1 #need to replace with an actual val
            for i in range(len(self.plates)):
                p = self.plates[i]
                if p.timeBuffer != 0:
                    # hasn't been seen, update count
                    p.timeBuffer += 1
                    if p.timeBuffer == kill_threshold:
                        #kill plate
                        self.kill_plate(i)
            return


            
        
    # Input from prediction/errorchecking?
    # unsure
    # def predictionInput(self, input):
    #     for i in input:
    #         self.plates[i].updateVelocity(input[i].velocity)
    #     return

    # pseudo greedy (?)
    # newPlate is the new armor plate we are trying to associate
    def assign_plate(self, newPlate, plates) -> int: 
        if newPlate == None or plates == None:
            #error check
            return -2
        predicted = newPlate.getNextPosition()
        shortest_dist = float('inf')
        shortest_plate = -1 # keeping the index, not decided what to do with the closest plate

        margin_of_err = 5 # this is some random number, we need to finetune this later

        for i in range(len(plates)):
            dist = self.get_distance(self, newPlate, plates[i])
            if dist < shortest_dist:
                shortest_plate = i
                shortest_dist = dist

        if shortest_dist > margin_of_err:
            # need to assign a new plate
            return -1
        return shortest_plate


    """
    Get_Distance returns the distance in pixels between two points
    input is two 3x1 matrices, each representing a point in (x,y,z)
    Output is a float which is the distance between the two objects 
    """
    def get_distance(self, point_one, point_two)-> float:

        np.sqrt(np.dot())
        
        return np.sqrt(np.pow((point_one[0]-point_two[0]),2)\
            +np.pow((point_one[1]-point_two[1]),2)\
            +np.pow((point_one[2]-point_two[2]),2))
    
    #log all of the armor plates at the very end of the program before all the plates are deleted (genocide D:)
    def kill_all(self, index):
        for plate in self.plates:
            plate.writeToHistory(self.objectLogOutput)
            self.kill_plate(self.plates.index(plate))
        self.objectLogOutput.close()
        return

    #logs and removes a single plate
    def kill_plate(self, index):
        self.plates[index].writeToHistory(self.objectLogOuput)
        self.plates.remove(index)
