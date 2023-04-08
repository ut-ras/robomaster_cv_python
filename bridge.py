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
    #bounding_boxes is a list of bounding_box objects
    bounding_boxes, color_image = mod.get_bounding_boxes()
    print(bounding_boxes)
    oLog.boxesInput(bounding_boxes, time.time())
    #feed bbo into depth calculation
    if len(bounding_boxes) > 0:
        dc.set_all_bounding_box_depth_values(bounding_boxes)
        print(color_image.shape)
        #feed bounding_boxes into update
        #print(color_frame)
        oLog.boxesInput(bounding_boxes, time.time())
        #color_image = np.asanyarray(color_frame.get_data())

        for i in range(len(bounding_boxes)):
            color_image = cv2.rectangle(color_image, bounding_boxes[i].__x_center__-(bounding_boxes[i].__width__/2), bounding_boxes[i].__y_center__-(bounding_boxes[i].__height__/2), bounding_boxes[i].__x_center__, bounding_boxes[i].__y_center__)
    cv2.namedWindow('Image',cv2.WINDOW_AUTOSIZE)
    cv2.imshow('Image', color_image)
    cv2.waitKey(1)
    #target select  the object
    sel = targetSel.selectTarget(oLog)

    #send to UART
    #sendtoUART()

    print(time.time()-prevTime) 
    prevTime = time.time()