import torch
import cv2
import sys
sys.path.append("../")
from object_log import bounding_box
from depth_calculation import depth_calculation as dp

class Model:
    def __init__(self, path):
        self.model = torch.hub.load(
            "yolov5",
            "custom",
            path=path,
            source="local",
        )


    def run():
        pass
        
    def get_bounding_boxes(self):
        color_image, depth_image = dp.get_color_depth_image()
        pred = self.model(color_image, size=720)
        xy_df = pred.pandas().xyxy[0]
        plates = xy_df[xy_df['name'].str.contains("Plate")]
        if(len(plates) > 0):
            plates = plates.reset_index()
            plates["xCenter"] = (plates['xmin'] + plates['xmax']) / 2
            plates["yCenter"] = (plates['ymin'] + plates['ymax']) / 2
            plates["width"] = (plates['xmax'] - plates['xmin'])
            plates["height"] = (plates['ymax'] - plates['ymin'])
            print(plates["xCenter"])
            print(plates["yCenter"])
            print(plates["width"])
            print(plates["height"])

            return [bbox for bbox in plates.apply(lambda x: bounding_box.BoundingBox(*x[['xCenter', 'yCenter', 'width', 'height']]), axis=1)], color_image, depth_image
        else:
            return [], color_image, depth_image