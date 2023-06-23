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

    def __init__(self):
        self.plates = ap.ArmorPlate()
        # for i in range(4):
        #     armor_plate = ap.ArmorPlate()
        #     armor_plate.set_id(i)
        #     self.plates.append(armor_plate)

    def get_plates(self):
        return self.plates
    
    def select_target(self, boundingbox_list):
        lowest_distance_idx = -1
        lowest_depth = float('inf')
        for i in range(len(boundingbox_list)):
            current_depth = boundingbox_list[i]
            if(current_depth < lowest_depth):
                lowest_depth = current_depth
                lowest_distance_idx = i
        
        self.plates.update_box(boundingbox_list[lowest_distance_idx])
        return self.plates.get_position(), self.plates.get_velocity(), self.plates.get_acceleration()


    #associate boxes with armor plates
    def assoc_boxes(self, boundingbox_list):
        self.plates.update_box(boundingbox_list[0])
        # self.update_predicted_positions()
        # for i in range(len(boundingbox_list)):
        #     plate_index = self.assign_plate(boundingbox_list[i], self.plates) #index of matching plate
        #     # if plate_index == -1: #if there is no association error handling
        #     #     print("could not find space for plate")
        #     self.plates[plate_index].update_box(boundingbox_list[i])
        
    
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
        # firstEmptyIndex = -1

        for i in range(len(self.plates)):
            # if firstEmptyIndex == -1 and not self.plates[i].get_active_status():     #gets first not active index. Useful to do it here to avoid an extra for loop later
            #     firstEmptyIndex = i
            #     continue
            # if not self.plates[i].get_seen_this_iter():     #if we have already associated this plate with a prior bounding box this frame don't overwrite that association
            #     continue
            #calculate distance between plate predicted and box
            box_coord = box.get_coord()
            plate_coords = self.plates[i].get_next_position()
            diff_dist = np.power(np.abs(plate_coords['x_pos']-box_coord['x_pos']),2)
            + np.power(np.abs(plate_coords['y_pos']-box_coord['y_pos']),2)
            + np.power(np.abs(plate_coords['z_pos']-box_coord['z_pos']),2)
            
            #find lowest one
            # if diff_dist < np.power(margin_of_err,2) and diff_dist < least_dist: 
            if diff_dist < least_dist:
                least_dist = diff_dist
                lowest_index = i
            
        #if none of them are close enough, go return an inactive plate
        # if lowest_index == -1:  #no currently active plates to bound to, return first empty index if any (returns -1 if no available plate which we can handle elsewhere ig)
        #     return firstEmptyIndex
        return lowest_index

        
            


            


    
    

