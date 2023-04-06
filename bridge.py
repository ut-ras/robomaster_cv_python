import pyrealsense2 as rs

from depth_calculation import depth_calculation as dc
from object_log import objectlog as ol

def init():
    dc.initialize_real_sense()
    oLog = ol.objectlog()

init()
while True:
    #oLog.boxesInput(Get_Bounding_box())
    print("LOL")