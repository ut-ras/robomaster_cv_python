from bounding_box import *
from armorplate import *

from objectlog import *
import numpy as np
import pytest

@pytest.fixture
def obj():
    return objectlog()


num = 9

def main():
    boxlistPrev = []
    boxlistNext = []
    for i in range(num):
        boxlistPrev[i] = BoundingBox()
        boxlistPrev[i].set_x_value(np.random(0,150))
        boxlistPrev[i].set_y_value(np.random(0,150))
        boxlistPrev[i].set_depth_value(np.random(0,150))
        boxlistPrev[i].set_height_value(np.random(0,150))
        boxlistPrev[i].set_width_value(np.random(0,150))
        boxlistPrev[i].set_time()

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

    
        

    
if __name__ == "__main__":
    main()