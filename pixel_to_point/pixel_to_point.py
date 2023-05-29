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
