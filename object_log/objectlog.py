import numpy as np
from object_log import armorplate as ap
import time

#TODO figure out correct values for this
#these are the constants used to calculate if a predicted position is out of bounds
#these should be updated with the actual values
# MAX_X = 150
# MAX_Y = 150
# MAX_Z = 50
# MIN_X = -1
# MIN_Y = -1
# MIN_Z = -1

#TODO margin of error used in 
margin_of_err = 0 # this is some random number, we need to finetune this later

"""
Object Log:
Purpose: System to hold current value of armor plate. 
Fields:
- plates: array of currently active armor plates (armor plate objects)
- id: ID associated with them
- Timestamp: The last timestamp they have.
Primary functions in pipeline:
- Take in input from depth/ML and associate armor plates
- Filter out old plates
- Write to a log file
"""
class objectlog:

    """
    Initialize function
    Runs in init() in main loop
    instantiates plates, ID var, and the timestamp variable. Opens output doc
    """
    def __init__(self):
        self.plates = []
        for i in range(4):
            armor_plate = ap.ArmorPlate()
            armor_plate.set_id(i)
            self.plates.append(armor_plate)
        
        #Python automatically creates the file with this name if it does not exist
        # self.objectLogOutput = open("ObjectLog.txt",'w')

    def get_plates(self):
        return self.plates
    """
    Input from depth
    Input: 
    - boxList: an array of of bounding box objects
    - timestamp: timestamp of the boundingBoxes
    the plates unless the closest distance is greater than some margin of error
    """
    def boxesInput(self, boxList, currentTime: float):
        for box in boxList:
            # greedy stuff done here \/
            assoc = self.assign_plate(box, self.plates) #index of matching plate
            # if assoc == -1:
            #     #new plate, not seen before
            #     if len(self.plates) < 9:
            #         # add new plate and we have space
            #         self.plates.append(new_armor)
            #         self.idAssign += 1
            #     else:
            #         #looking at more than 9 things
            #         print("need space")

            # elif assoc == -2:
            # elif assoc == -3:
            #     #out of range in x, y, or z (depth)
            #     print("out of range")
                
            # else:
            #     # add new plate to list of associated plates of plates[assoc]
            #     assoc_plate = self.plates[assoc]
            #     assoc_plate.addArmorPlate(new_armor, currentTime)
            #     assoc_plate.timeBuffer = 0
            #     self.idAssign += 1

            # bump up timer buffers and remove dead plates 
            # for i in range(len(self.plates)):
            #     p = self.plates[i]
            #     if p.timeBuffer != 0:
            #         # hasn't been seen, update count
            #         p.timeBuffer += 1
            #         if p.timeBuffer == kill_threshold:
            #             self.kill_plate(i)
            return
    

    def update_predicted_positions(self):
        for i in range(self.plates):
            self.plates[i].predict_position(time.time())
            predicted_position = self.plates[i].get_next_position()

    """
    Takes in an armor plate, makes sure that it is valid and not predicted to be out of
    range, and then associates it with the armor plate whose predicted position is closest
    to the position of this armor plate
    """
    def assign_plate(self, box) -> int:
        


            


    
    

