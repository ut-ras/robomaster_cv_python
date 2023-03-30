from bounding_box import *
from armorplate import *

from objectlog import *
import numpy as np
import pytest
import time

log = objectlog(0)
box = bounding_box()
armor = armorplate()

@pytest.fixture
def objectLogFixture():
    return log(time.localtime(time.time()))

@pytest.fixture
def boundingBoxFixture():
    return box()

@pytest.fixture
def armorplateFixture():
    return armor()

# parameters for previous bounding boxes
prev_params = [
    (100, 100, 25, 10, 10, 1),
    (50, 50, 25, 10, 10, 1),
    (75, 75, 25, 10, 10, 1)
]

num = 3
boxlistPrev = []
boxlistNext = []

boxVelocities = []
boxAccelerations = []

deltaTime = 1000 # time between boxListPrev and boxListNext in ms

# create global list of bounding boxes for testing purposes
def main():

    # generate a list of previous boxes
    makeBoxes(boxlistPrev, prev_params)

    for i in range(num):
        print(boxlistPrev[i].get_x_value())
        print('\n')
        print(boxlistPrev[i].get_y_value())
        print('\n')
        print(boxlistPrev[i].get_depth_value())
        print('\n')
        print(boxlistPrev[i].get_height_value())
        print('\n')
        print(boxlistPrev[i].get_width_value())
        boxlistPrev[i].print_time()
        print('\n')

# Hardcoding bounding boxes for temporary use
def makeBoxes(box_list, box_params):
    for i in range(0, num):
        box_list[i].append(BoundingBox())
        box_list[i].set_x_value(box_params[0])
        box_list[i].set_y_value(box_params[1])
        box_list[i].set_depth_value(box_params[2])
        box_list[i].set_height_value(box_params[3])
        box_list[i].set_width_value(box_params[4])
        box_list[i].set_time(box_params[5])


    boxVelocities[0] = [-25, -25, 0]
    boxAccelerations[0] = [-10, -10, 0]
    
    boxlistNext.append(BoundingBox())
    boxlistNext[0].set_x_value(75)
    boxlistNext[0].set_y_value(75)
    boxlistNext[0].set_depth_value(25)
    boxlistNext[0].set_height_value()
    boxlistNext[0].set_width_value()
    boxlistNext[0].set_time()


    boxVelocities[1] = [10, 20, 0]
    boxAccelerations[1] = [30 ,-2, 0]

    boxlistNext.append(BoundingBox())
    boxlistNext[1].set_x_value()
    boxlistNext[1].set_y_value()
    boxlistNext[1].set_depth_value()
    boxlistNext[1].set_height_value()
    boxlistNext[1].set_width_value()
    boxlistNext[1].set_time()
    

    boxVelocities[2] = [40, 2, 0]
    boxAccelerations[2] = [10 ,20, 0]

    boxlistNext.append(BoundingBox())
    boxlistNext[2].set_x_value()
    boxlistNext[2].set_y_value()
    boxlistNext[2].set_depth_value()
    boxlistNext[2].set_height_value()
    boxlistNext[2].set_width_value()
    boxlistNext[2].set_time()

    return


# Test something here
def test_basic(obj):
    # TODO: write test
    one = 1
    assert one == 1
    pass

def test_boxesInputInit(obj):#this doesn't test out of bounds plates yet (modify later)
    log = objectlog(0)
    log.boxesInput(boxlistPrev,10)
    plates = log.get_plates
    assert(len(plates) == 3)

    
    #this part is to check ids (see if they are unique)
    isunique  = 1  
    for i in (0,len(plates)):
        for j in range(1, len(plates)):
            if isunique==1 and plates[i].getID()==plates[j].getID():
                isUnique = 0

    assert(isUnique==1)
    
    
    log.boxesInput(boxlistNext, 10,20)
    pass

# check for correct assignment of IDs to each armor plate
@pytest.mark.parametrize("expectedID", [
    (0),
    (1),
    (2)
])
def test_assignID(expectedID):
    log = objectlog(0)  # current timestamp set to 0
    log.boxesInput(boxlistPrev, 10) # 10 is random value for current time
    plates = log.get_plates
    for i in range(0, len(plates)):
        assert plates[i].getID() == expectedID




#Should test that the object log accurately assigns plates
def test_assignPlate(obj):
    plates = objectlog.get_plates
    objectlog.assign_plate(boxlistPrev[0])
    objectlog.assign_plate(boxlistPrev[1])
    objectlog.assign_plate(boxlistPrev[2])
 
    pass
#Should test that the object log is killing dead plates
def test_killPlate(obj):
    pass

def test_killAll(obj):
    pass



# Test that objectlog can accurately match the right bounding box to the right armor plate after delta_t time
def test_predict(obj):
    # TODO: write test
    pass


if __name__ == "__main__":
    main()