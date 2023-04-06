import sys
import pyrealsense2 as rs

from depth_calculation import depth_calculation as dc
from object_log import objectlog as ol
import target_selection as ts
import prediction as p
import time

def init():
    #initialize ML model
    dc.initialize_real_sense()
    oLog = ol.objectlog()

init()
while True:
    #oLog.update(Get_Bounding_box())
    #grab ML bbo
    temp = None
    #feed bbo into depth calculation
    
    #feed temp into update
    ol.boxesInput(temp, time.time())

    #target select the object
    sel = ts.selectTarget(ol)

    #send to UART
    #sendtoUART()

    
    print("LOL")