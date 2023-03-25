from bounding_box import *
from armorplate import *

from objectlog import *
import numpy as np
import pytest
import time

@pytest.fixture
def objectLogFixture():
    return objectlog(time.localtime(time.time()))

@pytest.fixture
def boundingBoxFixture():
    return BoundingBox()

@pytest.fixture
def armorplateFixture():
    return armorplate()

num = 3
boxlistPrev = []
boxlistNext = []

boxVelocities = []
boxAccelerations = []

deltaTime = 1000 # time between boxListPrev and boxListNext in ms

# create global list of bounding boxes for testing purposes
def main():
    
    makeBoxes(boxlistPrev)

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
def makeBoxes(boxlistPrev):
    # Bounding Box 1 - On the edge
    boxlistPrev.append(BoundingBox())
    boxlistPrev[0].set_x_value(100)
    boxlistPrev[0].set_y_value(100)
    boxlistPrev[0].set_depth_value(25)
    boxlistPrev[0].set_height_value(10)
    boxlistPrev[0].set_width_value(10)
    boxlistPrev[0].set_time(1)

    boxVelocities[0] = [-25, -25, 0]
    boxAccelerations[0] = [-10, -10, 0]
    
    boxlistNext.append(BoundingBox())
    boxlistNext[0].set_x_value(75)
    boxlistNext[0].set_y_value(75)
    boxlistNext[0].set_depth_value(25)
    boxlistNext[0].set_height_value()
    boxlistNext[0].set_width_value()
    boxlistNext[0].set_time()

    # Bounding Box 2 - Within Range 
    boxlistPrev.append(BoundingBox())
    boxlistPrev[1].set_x_value(50)
    boxlistPrev[1].set_y_value(50)
    boxlistPrev[1].set_depth_value(25)
    boxlistPrev[1].set_height_value(10)
    boxlistPrev[1].set_width_value(10)
    boxlistPrev[1].set_time(1)

    boxVelocities[1] = [10, 20, 0]
    boxAccelerations[1] = [30 ,-2, 0]

    boxlistNext.append(BoundingBox())
    boxlistNext[1].set_x_value()
    boxlistNext[1].set_y_value()
    boxlistNext[1].set_depth_value()
    boxlistNext[1].set_height_value()
    boxlistNext[1].set_width_value()
    boxlistNext[1].set_time()
    
    # Bounding Box 3 - Outside Range
    boxlistPrev.append(BoundingBox())
    boxlistPrev[2].set_x_value(200)
    boxlistPrev[2].set_y_value(200)
    boxlistPrev[2].set_depth_value(100)
    boxlistPrev[2].set_height_value(10)
    boxlistPrev[2].set_width_value(10)
    boxlistPrev[2].set_time(1)

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

   
    
# Test that objectlog can accurately match the right bounding box to the right armor plate after delta_t time
def test_predict(obj):
    # TODO: write test
    pass


if __name__ == "__main__":
    main()