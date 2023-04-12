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
    
    #Retrieves bounding boxes
    bounding_boxes, color_image, depth_image = mod.get_bounding_boxes()
    oLog.boxesInput(bounding_boxes, time.time())
    #feed bbo into depth calculation
    if len(bounding_boxes) > 0:
        dc.set_all_bounding_box_depth_values(depth_image, bounding_boxes)
        print("Hello")
        #feed bounding_boxes into update
        oLog.boxesInput(bounding_boxes, time.time())

    #For drawing the bounding boxes
        for i in range(len(bounding_boxes)):
            pt1 = (int((bounding_boxes[i].get_x_value()-bounding_boxes[i].get_width())/2), int((bounding_boxes[i].get_y_value()-bounding_boxes[i].get_height())/2))
            pt2 = (int((bounding_boxes[i].get_x_value()+bounding_boxes[i].get_width())/2), int((bounding_boxes[i].get_y_value()+bounding_boxes[i].get_height())/2))
            color_image = cv2.rectangle(color_image, color=(255,0,0),pt1=pt1, pt2=pt2, thickness=4)
    cv2.namedWindow('Image',cv2.WINDOW_AUTOSIZE)
    cv2.imshow('Image', color_image)
    cv2.waitKey(1)

    #target select  the object
    sel = targetSel.selectTarget(oLog)

    #send to UART
    #sendtoUART()

    #Prints out delta t
    # print(time.time()-prevTime) 
    # prevTime = time.time()

 