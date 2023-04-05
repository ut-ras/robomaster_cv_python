from armorplate import *
import numpy as np
import bounding_box

#TODO figure out correct values for this
#these are the constants used to calculate if a predicted position is out of bounds
#these should be updated with the actual values
MAX_X = 150
MAX_Y = 150
MAX_Z = 50
MIN_X = -1
MIN_Y = -1
MIN_Z = -1

#TODO figure out smallest allowed order
MIN_AREA = 10

#TODO margin of error used in 
margin_of_err = 5 # this is some random number, we need to finetune this later

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
        self.idAssign = 0
        
        #Python automatically creates the file with this name if it does not exist
        self.objectLogOuput = open("ObjectLog.txt",'w')


    """
    Input from depth
    Input: 
    - boxList: an array of of bounding box objects
    - timestamp: timestamp of the boundingBoxes
    the plates unless the closest distance is greater than some margin of error
    """
    def boxesInput(self, boxList, timestamp, currentTime):

        #self.timeStamp = timestamp #timestamp is used for finding the delta t to do the kinematics. This needs to change, it is wrong

        # add all bounding boxes to plates if plates is empty
        if len(self.plates) == 0:
            for i in range(len(boxList)):
                #check size of bounding box, if too small pass this iteration
                if not self.size_check(boxList[i]):
                    continue

                #error check the bounding box parameters
                if((boxList[i].get_x_value() < 0) or (boxList[i].get_y_value() < 0) or (boxList[i].get_depth() < 0) or (boxList[i].get_height() < 0) or (boxList[i].get_width() < 0)):
                    #bounding box is invalid
                    return -1

                newPlate = ArmorPlate(boxList[i], self.idAssign)

                #commented out for syntax checking COMMENT BACK IN FOR ACTUAL THING
                #***************************************************************
                kinematic_Update(newPlate.getPosition[0], newPlate.getPosition[1], newPlate.getPosition[2])
                kinematic_predict(currentTime - timestamp)
                newPlate.updateVA(getVA())
                #***************************************************************

                self.plates.append(newPlate)
                self.idAssign += 1 
        else:
            for i in range(len(boxList)):
                #check size of bounding box, if too small pass this iteration
                if not self.size_check(boxList[i]):
                    continue 

                #error check the bounding box parameters
                if((boxList[i].get_x_value() < 0) or (boxList[i].get_y_value() < 0) or (boxList[i].get_depth() < 0) or (boxList[i].get_height() < 0) or (boxList[i].get_width() < 0)):
                    #bounding box is invalid
                    return -1

                
                new_armor = ArmorPlate(boxList[i], self.idAssign)
                new_armor.predictPosition(currentTime)

                # greedy stuff done here \/
                assoc = self.assign_plate(new_armor, self.plates) #index of matching plate
                if assoc == -1:
                    #new plate, not seen before
                    if len(self.plates) < 9:
                        # add new plate and we have space
                        self.plates.append(new_armor)
                    else:
                        #looking at more than 9 things
                        print("need space")

                elif assoc == -2:
                    #panic, something is null
                    print("panic")

                elif assoc == -3:
                    #out of range in x,y, or z
                    print("out of range")
                    
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

    def size_check(self, box) -> bool:
        #10 is an abitrary num
        if box.get_height() * box.get_width() < MIN_AREA:
            return False
        return True
            
        
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
        #This variable accounts for the plate's position being the center of the bounding box
        #maybe factor in the plate boundaries into the out of range calculations
        #plate_radius = newPlate.getBoundingBox().get_width() / 2
        shortest_dist = float('inf')
        shortest_plate = -1 # keeping the index, not decided what to do with the closest plate

        if(((predicted[0] + margin_of_err) > MAX_X) or ((predicted[1] + margin_of_err) > MAX_Y) or ((predicted[2] + margin_of_err) > MAX_Z)
           or ((predicted[0] - margin_of_err) < MIN_X) or ((predicted[1] - margin_of_err) < MIN_Y) or ((predicted[2] - margin_of_err) < MIN_Z)):       
            #if it is out of range, return early
            return -3

        for i in range(len(plates)):
            dist = self.get_distance(self, newPlate, plates[i])
            if dist < shortest_dist:
                shortest_plate = i
                shortest_dist = dist

        if shortest_dist > margin_of_err:
            # need to assign a new plate
            return -1
        return shortest_plate # this returns an index, but this could change to 
    # an actual refernce to the plate ¯\_(ツ)_/¯


    """
    Get_Distance returns the distance in pixels between two points
    input is two 3x1 matrices, each representing a point in (x,y,z)
    Output is a float which is the distance between the two objects 
    """
    def get_distance(self, point_one, point_two)-> float:
        
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
    
    def get_plates(self):
        #used for debugging 
        return self.plates

    #logs and removes a single plate
    def kill_plate(self, index):
        self.plates[index].writeToHistory(self.objectLogOutput)
        self.plates.remove(index)
