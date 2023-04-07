import torch
import cv2
import sys
sys.path.append("../")
from object_log import bounding_box


class Model:
    def __init__(self, path):
        self.model = torch.hub.load(
            "yolov5",
            "custom",
            path=path,
            source="local",
        )
        self.cam_feed = cv2.VideoCapture(1)
        self.cam_feed.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cam_feed.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        self.image_attempts = 5


    def run():
        pass
        
    def get_bounding_boxes(self):
        for _ in range (self.image_attempts):
            ret, img = self.cam_feed.read()   
            if ret:
                break
        if not ret:
            print("Error in accessing camera")
            return [], img
        pred = self.model(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), size=480)
        xy_df = pred.pandas().xyxy[0]
        plates = xy_df[xy_df['name'].str.contains("Plate")]
        if(len(plates) > 0):
            plates = plates.reset_index()
            plates["xCenter"] = plates['xmin'] + plates['xmax'] / 2
            plates["yCenter"] = plates['ymin'] + plates['ymax'] / 2
            plates["width"] = plates['xmax'] - plates['xmin']
            plates["height"] = plates['ymax'] - plates['ymin']
            print(plates["xCenter"])
            print(plates["yCenter"])
            print(plates["width"])
            print(plates["height"])

            return [bbox for bbox in plates.apply(lambda x: bounding_box.BoundingBox(*x[['xCenter', 'yCenter', 'width', 'height']]), axis=1)], img
        else:
            return [], img