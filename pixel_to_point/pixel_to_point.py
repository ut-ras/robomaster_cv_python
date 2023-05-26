import pyrealsense2.pyrealsense2 as rs

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
    _intrinsics.ppx = intrinsics.K[2]
    _intrinsics.ppy = intrinsics.K[5]
    _intrinsics.fx = intrinsics.K[0]
    _intrinsics.fy = intrinsics.K[4]
    _intrinsics.model  = rs.distortion.none

    _intrinsics.coeffs = [i for i in intrinsics.D]

    result = rs.rs2_deproject_pixel_to_point(_intrinsics, [x, y], depth)
    return result[2], -result[0], -result[1]
