import sys
import pyrealsense2 as rs
import time

from depth_calculation import depth_calculation as dc
from object_log import objectlog as ol
from target_selection import Target_Selection as ts
from ML import Model as m

dc.initialize_real_sense()
oLog = ol.objectlog()
targetSel = ts.targetSelection()
mod = m.Model('../Robomaster_CV/ML/best.pt')
while True:
    oLog.boxesInput(mod.get_bounding_boxes(), time.time())
    #grab ML bbo
    temp = None
    #feed bbo into depth calculation
    dc.get_all_color_image_(temp)
    #feed temp into update
    oLog.boxesInput(temp, time.time())

    #target select  the object
    sel = targetSel.selectTarget(oLog)

    #send to UART
    #sendtoUART()

    
    print("LOL")