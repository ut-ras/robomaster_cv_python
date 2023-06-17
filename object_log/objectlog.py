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

    def get_plates(self):
        return self.plates
    """
    Input from depth
    Input: 
    the plates unless the closest distance is greater than some margin of error
    """
    def input_boxes(self, boundingbox_list):
        self.update_predicted_positions()
        for i in range(len(boundingbox_list)):
            assoc = self.assign_plate(boundingbox_list[i], self.plates) #index of matching plate
            return
    
    #using kinematics, predict where the enemy robot would be between the last time it was seen and now
    def update_predicted_positions(self):
        for i in range(len(self.plates)):
            self.plates[i].predict_position(time.time())



    """
    Takes in a bounding box and then associates it with the armor plate whose predicted position is closest
    to the position of this armor plate
    """
    def assign_plate(self, box) -> int:
        lowest_index = -1
        least_dist = float('inf')
        for i in range(len(self.plates)):
            #calculate distance between plate predicted and box
            box_coord = box.get_coord()
            plate_coords = self.plates[i].get_next_position()
            diff_dist = abs(plate_coords['x_pos']-box_coord['x_pos'])
            + abs(plate_coords['y_pos']-box_coord['y_pos'])
            + abs(plate_coords['z_pos']-box_coord['z_pos'])
            
            #find lowest one
            if(diff_dist < least_dist):
                least_dist = diff_dist
                lowest_index = i
            
        
            


            


    
    

