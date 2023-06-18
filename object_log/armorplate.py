if __name__ == "__main__":
    import bounding_box as bb
    import prediction as pred
    
import numpy as np
from prediction import prediction as pred
from bounding_box import bounding_box as bb

class ArmorPlate:
    MAX_ASSOC_PLATES = 5
    """
    A object of an armor plate object that is typically associated with what is seen.
    Object should be initialized from the outputs of the machine learning model and has parameters such as position, velocity, depth, etc.
    
    Armor plate object is used in object log
    """

    def __init__(self):
        """
        Initializes the armor plate
        Rundown of the fields of the object:
        position, holds the position of the bounding box in a x, y, z system (camera relative)
        velocity, last velocity of the target
        acceleration, last acceleration of the target
        boundingbox, boundingbox object
        id, id of the armor plate (for debugging purposes)
        activity, boolean on the plate on if it is currently alive
        
        """
        self.position = 0 #boundingbox.get_position()
        self.velocity = None 
        self.acceleration = None
        self.boundingbox = None # bounding box object
        self.id = None
        self.armor_plate_active = False
        self.plate_seen_this_iter = False
        # self.time_buffer = 0
        self.next_position = None
        self.last_time = None
        self.assoc_boxes = [] # used for history
        self.kf = pred.Prediction()
    
    ##
    ## Getters
    ##

    # {"x_pos":0,
    #  "y_pos":0,
    #  "z_pos":0}
    def get_position(self):
        return self.position 
    
    # {"x_vel":0,
    #  "y_vel":0,
    #  "z_vel":0}
    def get_velocity(self):
        return self.velocity
    
    # {"x_acc":0,
    #  "y_acc":0,
    #  "z_acc":0}

    def get_acceleration(self):
        return self.acceleration
    
    def get_id(self):
        return self.id 
    
    #most recent bounding box
    def get_boundingbox(self):
        return self.boundingbox
    
    def get_active_status(self):
        return self.armor_plate_active
    
    def get_time_buffer(self):
        return self.time_buffer
    
    def get_next_position(self):
        return self.next_position
    
    #last recorded time of bounding box
    def get_last_time(self):
        return self.last_time
    
    def get_assoc_boxes(self):
        return self.assoc_boxes
    
    def get_seen_this_iter(self):
        return self.plate_seen_this_iter
    
    ##
    ## Setters
    ##

    def set_position(self, position):
        self.position = position

    def set_vel(self, vel:dict):
        self.velocity = vel
    
    def set_acc(self, acc):
        self.acceleration = acc

    def set_boundingbox(self, bounding):
        self.boundingbox = bounding

    def set_id(self, id):
        self.id = id
        
    def set_armor_plate_active(self, status):
        self.armor_plate_active = status

    def set_next_position(self, position):
        self.next_position = position
        
    def set_last_time(self, last_time):
        self.last_time = last_time

    def set_seen_iter(self, status: bool):
        self.plate_seen_this_iter = status

    def add_assoc_boxes(self, box):
        self.assoc_boxes.append(box)

    # Velocity and acceleration are sets of three values.
# """
# Input from Armorplate
# Input:
#     - Current Velocity vector of armor plate
#     - Current Acceration vector of armor plate
# Output:
#     - Predicated Velocity vector of armor plate
#     - Predicted Acceration vector of armor plate
# """
    def update_VA(self):
        vel_dict, acc_dict = self.kf.getVA()
        self.set_vel(vel_dict)
        self.set_acc(acc_dict)

    """
    predictPosition uses position, velocity, and delta time to predict where armor plate have moved to since the last check;
    this method is subject to change depending on how PVA is implemented (currently assumed to be world positions)
    assumes that time is being kept track of in per second units while velocity/acceleration are per millisecond units.
    """
    def predict_position(self, currentTime: float):
        delta_t = currentTime - self.get_last_time()
        self.set_next_position(self.get_next_position() + (np.multiply(self.get_velocity(), delta_t)) + (np.multiply(self.get_acceleration(), np.exp(delta_t, 2) / 2))) # kinematics :D
        self.set_seen_iter(False)

    """
    updateBox takes in a boundingBox and updates all the status variables involved
    """
    def update_box(self, newBox: bb):
        self.set_boundingbox(newBox)
        self.set_position(newBox.get_position())
        self.set_armor_plate_active(True)
        self.set_seen_iter(True)
        delta_t = newBox.get_time() - self.get_last_time()
        self.set_last_time(newBox.get_time())

        #TODO: Run kalman filter here to get pva?
        self.kf.kinematicPredict(delta_t)
        position_array = np.array([newBox.get_position().get("x_pos"),
                                   newBox.get_position().get("y_pos"),
                                   newBox.get_position().get("z_pos"),])
        self.kf.kinematicUpdate(position_array)
        self.update_VA()


    # """
    # Given an armorplate we have associated in objectlog, 
    # add to a list of associated armor plates.
    # """
    # def addArmorPlate(self, new_plate: bb, currentTime: float):
    #     #add to data structure
    #     if len(self.assoc_plates) == self.max_assoc_plates:
    #         self.assoc_plates.pop()
    #     self.assoc_plates.insert(0,new_plate)
    #     self.kf.kinematicUpdate(new_plate.getPosition())
    #     self.lastTime = currentTime

    #     return

    """
    # writeToHistory writes the ID, position, activity, and lastTime variables to a text file in a
    # formatted manner. This function is currently not in use for testing purposes
    # """
    # def writeToHistory(self, historyFile):
    #     # historyFile = open("pathhere",'a')
    #     historyFile.write("ID: {} Position: {} Activity: {} LastTime: {}\n"\
    #                       .format(self.id, self.position, self.activity, self.lastTime))
    #     # historyFile.close()