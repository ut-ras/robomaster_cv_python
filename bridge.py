import sys
import pyrealsense2 as rs

sys.path.append('/depth_calculation/')
sys.path.append('/object_log/')
sys.path.append('/target_secltion/')
sys.path.append('/Abc/')

import depth_calculation as dc
import object_log as ol
import target_selection as ts
import prediction as p

def init():
    dc.initialize_real_sense()
    oLog = ol.objectlog()


while True:
    init()
    #oLog.update(Get_Bounding_box())

