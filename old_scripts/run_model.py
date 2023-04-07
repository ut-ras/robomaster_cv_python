import torch
import glob
from bounding_box import BoundingBox
import cv2
import ttach as tta
from Model import Model

m = Model("./yolov5/runs/train/exp5/weights/best.pt")

for i in range(5):
    print(m.get_bounding_boxes())