import torch
import cv2
from kalmanfilter import KalmanFilter
import time

# function for getting center of armor plate, wont work if there are multiple objects detected
def getXY(xy):
    plates = xy[xy['name'].str.contains("Plate")]
    if(len(plates) > 0):
        plates = plates.reset_index()
        x = (plates['xmin'][0] + plates['xmax'][0]) / 2
        y = (plates['ymin'][0] + plates['ymax'][0]) / 2
        return (x,y)
    else:
        return None

# Models and Filters
model = torch.hub.load('yolov5/', 'custom', 
    path='yolov5\\runs\\train\\trained_on_deblurred_images_only\\weights\\best.pt', 
    source='local')
kf = KalmanFilter()

# Setup camera
cam_feed = cv2.VideoCapture(0)
cam_feed.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cam_feed.set(cv2.CAP_PROP_FRAME_HEIGHT, 750)


while True:
    # yolo detection
    ret, img = cam_feed.read()   
    pred = model(cv2.cvtColor(img, cv2.COLOR_BGR2RGB), size=640)


    output_img = pred.render()[0]
    # Kalman filter prediction
    plate_loc = getXY(pred.pandas().xyxy[0])
    if(plate_loc is not None):
        x, y = plate_loc

        xpred, ypred = kf.predict(x, y)

        # draw the circle
        cv2.circle(output_img, [int(xpred), int(ypred)], 10, color=(0, 165, 255), thickness=3)
        
    # display the frame
    cv2.imshow("", cv2.cvtColor(output_img, cv2.COLOR_RGB2BGR))     
    
    if (cv2.waitKey(1) & 0xFF == ord("q")) or (cv2.waitKey(1)==27):
        break


cam_feed.release()
cv2.destroyAllWindows()
