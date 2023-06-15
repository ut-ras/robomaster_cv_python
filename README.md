# UT RoboMaster CV - Stampede

UT Stampede RoboMaster CV Repository

Code is written for BeagleBone AI-64

For questions about the BeagleBone AI-64, our code, history of development on the BeagleBone AI-64 for a full RoboMaster CV system, and tips on getting started, documentation, etc. please contact Paul Han (email: pauljhan@utexas.edu Discord: @bornhater)

## Code Organization

Our CV system is organized into subsystems that cover various tasks necessary for a full RoboMaster CV system

Subsystems are listed below, in order of being called in *main.py* which is a continuous loop, running our system:

(**NOTE**: Almost none of these tasks are performed inside main.py. These functions abstract out the tasks at hand and are merely called in main.py, making debugging and development significantly easier. Unit tests will be added in the future)

**Image and Depth System** - Retrieving images and depth values (how far away is the armor plate in meters) from RealSense camera through Python wrapper

**Object Detection** - On the retrieved images, detect armor plates through the use of YOLOv5, object detection machine learning model

**Pixel To Point** - Convert image-relative x, y coordinate values (i.e on the image, from which pixel to which pixel is the detected armor plate) to world-relative coordinates values (in x, y, z coordinates, how many meters away is the armor plate)

**Object Log** - Map detected armor plates to specific robots for logging, target selection, and prediction purposes

**Prediction** - Using an unscented Kalman Filter, estimate the next position, current velocity, and current acceleration of the detected robot

**Communication** - Through UART, send x, y, z position, velocity, and acceleration values for ballistics purposes

