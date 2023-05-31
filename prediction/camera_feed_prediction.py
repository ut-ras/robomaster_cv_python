import pyrealsense2.pyrealsense2 as rs
'''
    x axis: horizontal plane
    y axis: vertical plane
    z axis: forward plane (depth values)

    follows realsense SDK convention
'''

# Converts pixel coordinates and depth to 3D coordinates (in meters)
# 
# param[in]: x - pixel coordinate x
# param[in]: y - pixel coordinate y
# param[in]: depth - depth in meters
# param[in]: intrinsics - intrinsics of the camera
# return: x, y, z - 3D coordinates in meters
# From: https://medium.com/@yasuhirachiba/converting-2d-image-coordinates-to-3d-coordinates-using-ros-intel-realsense-d435-kinect-88621e8e733a
def convert_pixel_and_depth_to_point(x, y, depth, intrinsics):
    _intrinsics = rs.intrinsics()
    _intrinsics.width = intrinsics.width
    _intrinsics.height = intrinsics.height
    _intrinsics.ppx = intrinsics.ppx
    _intrinsics.ppy = intrinsics.ppy
    _intrinsics.fx = intrinsics.fx
    _intrinsics.fy = intrinsics.fy
    _intrinsics.model  = rs.distortion.none

    _intrinsics.coeffs = intrinsics.coeffs

    result = rs.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)
    return -result[1], -result[0], result[2]


# TODO: Somehow pull from feature/pixel-to-point so you don't have to do the above?? (or it this integrated with the following)
import prediction as p

# Create Prediction Object
predict = p.Prediction()

# TODO: convert timestamp of realsense camera to milliseconds (?)
# Time step in milliseconds
dt = 0.1    # Arbitrary time step value

# TODO: Either retrieve data directly from the camera or have arguments passed into the function
# Returns more accurate estimate of the [x,y,z] position of object in meters
def predicted_position():
    # Retrieve positional data from camera feed (noisy)
    x = 0   # x pixel coordinate 
    y = 0   # y pixel coordinate 
    d = 0   # depth pixel coordinate 
    i = 0   # intrinsics of camera << could possibly global value
    position = convert_pixel_and_depth_to_point(x, y, d, i)
 
    predict.kinematicPredict(dt)    # predict position
    predict.kinematicUpdate([position[0], position[1], position[2]])    # estimate object position
    position = predict.getPredictedPos()    # retrieve position
    return position

