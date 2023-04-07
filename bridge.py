import sys
import pyrealsense2 as rs
import time
import cv2
import numpy as np

from depth_calculation import depth_calculation as dc
from object_log import objectlog as ol
from target_selection import Target_Selection as ts
from ML import Model as m

dc.initialize_real_sense()
oLog = ol.objectlog()
targetSel = ts.targetSelection()
mod = m.Model('../Robomaster_CV/ML/best.pt')
prevTime = 0
while True:
    
    #grab ML bbo
    temp = mod.get_bounding_boxes()
    print(temp)
    oLog.boxesInput(temp, time.time())
    #feed bbo into depth calculation
    color_frame = dc.get_all_color_image_(temp)
    #feed temp into update
    oLog.boxesInput(temp, time.time())
    color_image = np.asanyarray(color_frame.get_data())

    for i in range(len(temp)):
        color_image = cv2.rectangle(color_image, temp[i].__x_center__-(temp[i].__width__/2), temp[i].__y_center__-(temp[i].__height__/2), temp[i].__x_center__, temp[i].__y_center__)
    cv2.imshow('Image', color_image)
    cv2.waitKey(1)
    #target select  the object
    sel = targetSel.selectTarget(oLog)

    #send to UART
    #sendtoUART()

    print(time.time()-prevTime) 
    prevTime = time.time()