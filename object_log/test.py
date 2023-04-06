from armorplate import *
from bounding_box import *
from objectlog import *
from targetSelection import *
import numpy as np
import pytest
import time

# object log that persists across tests
hist_log = objectlog()

# box = BoundingBox()
# armor = ArmorPlate(box,1)

num = 3
boxlistPrev = []
boxlistNext = []

dt = 1000 # time between boxListPrev and boxListNext in ms

# parameters for previous bounding boxes
# x position, y position, depth, height, width, time
prev_params = [
    (100, 100, 25, 10, 10, dt),
    (50, 50, 25, 10, 10, dt),
    (75, 75, 25, 10, 10, dt)
]
# parameters for next bounding boxes (next frame)
# x position, y position, depth, height, width, time
next_params = [
    (101, 101, 25, 10, 10, 2*dt),
    (55, 55, 25, 10, 10, 2*dt),
    (76, 76, 25, 10, 10, 2*dt)
]

# bounding box VA
# x velocity, y velocity, z velocity, x acceleration, y acceleration, z acceleration
param_VA = [
    (1, 1, 0, 0, 0, 0),
    (5, 5, 0, 0, 0, 0),
    (1, 1, 0, 0, 0, 0)
]

@pytest.fixture
def obj_log():
    return hist_log(time.localtime(time.time()))

#What is this for \/

# @pytest.fixture
# def boundingBoxFixture():
#     return box()

# @pytest.fixture
# def armorplateFixture():
#     return armor()

# hardcoding bounding boxes for temporary use
def makeBoxes(box_list, box_params):
    for i in range(0, num):
        box = BoundingBox()
        box.set_x_value(box_params[i][0])
        box.set_y_value(box_params[i][1])
        box.set_depth(box_params[i][2])
        box.set_height(box_params[i][3])
        box.set_width(box_params[i][4])
        box.set_time() #time only takes in one argument
        box_list.append(box)

makeBoxes(boxlistPrev, prev_params)
makeBoxes(boxlistNext, next_params)



# Tests Needed (not comprehensize, please add more to this list as you see fit)
# Might have to test the robustness of the code later, add edge cases where needed
# X test that ID assignment is correct
# X test boxesInput (along with predicti position, assign plates and everythign in boxesInput)
# O test target selection algorithm
# X test killPlate
# X test killAllPlates



# check for correct assignment of IDs to each armor plate
@pytest.mark.parametrize("expectedID", [
    (0),
    (1),
    (2)
])

# test that ID is correctly assigned.
def test_assignID(expectedID):
    hist_log.boxesInput(boxlistPrev, dt, dt)
    plates = hist_log.get_plates()
    for i in range(0, len(plates)):
        assert plates[i].getID() == expectedID + i
    pass

test_assignID(0)

# test that bounding boxes are assigned correctly across time
# assumes that test_assign_ID has already run
def test_boxesInput():
    # set the velocity and acceleration values of the existing armor plates
    old_plates = hist_log.get_plates()
    for i in range(0, len(old_plates)):
        old_plates[i].updateVA(param_VA[i])

    hist_log.boxesInput(boxlistNext, dt,  2 * dt) # step forward by dt time
    new_plates = hist_log.get_plates

    assert len(new_plates) == 3 # if there are not 3 armor plates objects in object log, something went wrong...
    for i in range(0, len(new_plates)):
        assert new_plates[i].getPosition == [next_params[i][0], next_params[i][1], next_params[i][2]] #check if position of the armor plates are correct
    pass

test_boxesInput()


# test that the camera is able to a select a target.
def test_selectTarget():
    expectedTargetPosition = [next_params[0][0], next_params[0][1], next_params[0][2]] # manually set the target
    target = selectTarget(hist_log, hist_log.getCenter())
    assert target.getPosition == expectedTargetPosition
    pass

# test that takes a plateID argument and kills a single plate
def test_killPlate(plate_ID):
    plate_ID = 2    # hardcoding to remove the last element, but you can pass any plateID
    plates = hist_log.get_plates
    x = plates[plate_ID].getPosition[0]
    y = plates[plate_ID].getPosition[1]
    z = plates[plate_ID].getPosition[2]
    hist_log.kill_plate(plate_ID)
    for i in range(0, len(plates)): # checks the details of other plates to make sure we killed right plate
         assert(i < num)
         assert(x != plates[i].getPosition[0])
         assert(y != plates[i].getPosition[1])
         assert(z != plates[i].getPosition[2])
    pass


# test that checks if all plates murdered
def test_killAll():
    plates = hist_log.get_plates
    hist_log.kill_all
    assert(len(plates) == 0) # all have died :(
    pass

