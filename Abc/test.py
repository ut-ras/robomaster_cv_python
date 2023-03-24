from filterpy.kalman import KalmanFilter
import matplotlib.pyplot as plt
import numpy as np
import array as arr
import time
from filterpy.common import Q_discrete_white_noise
timePast = 0
# ap = np.array([0.0, 0.3, 1.2, 2.6999999999999997, 4.8, 7.5, 10.799999999999999, 14.7, 19.2, 24.3, 30.0, 36.3, 43.199999999999996, 50.699999999999996, 58.8, 67.5, 76.8, 86.7, 97.2, 108.3])
# aq = np.array([0.0, 0.8, 3.2, 7.2, 12.8, 20.0, 28.8, 39.2, 51.2, 64.8, 80.0, 96.80000000000001, 115.2, 135.20000000000002, 156.8, 180.0, 204.8, 231.20000000000002, 259.2, 288.8])
# ar = np.array([0.0, 0.8, 3.2, 7.2, 12.8, 20.0, 28.8, 39.2, 51.2, 64.8, 80.0, 96.80000000000001, 115.2, 135.20000000000002, 156.8, 180.0, 204.8, 231.20000000000002, 259.2, 288.8])

# delt = 0.1
f = KalmanFilter (dim_x=9, dim_z=3)
f.x = np.array([0., 0., 1., 0., 0., 1., 0., 0., 1.])
# f.F = np.array([[1.,    delt,   0.5*(delt**2),     0., 0., 0., 0., 0., 0.],
#                 [0.,    1.,     delt,               0., 0., 0., 0., 0., 0.],
#                 [0.,    0.,     1.,                 0., 0., 0., 0., 0., 0.],
#                 [0., 0., 0.,   1.,    delt,   0.5*(delt**2),   0., 0., 0.],
#                 [0., 0., 0.,    0.,    1.,     delt,            0., 0., 0.],
#                 [0., 0., 0.,    0.,    0.,     1.,              0., 0., 0.],
#                 [0., 0., 0., 0., 0., 0.,   1.,    delt,   0.5*(delt**2)],
#                 [0., 0., 0., 0., 0., 0.,    0.,    1.,     delt],
#                 [0., 0., 0., 0., 0., 0.,    0.,    0.,     1.]])
f.H = np.array([[1., 0., 0., 0., 0., 0., 0., 0., 0.],
                [0., 0., 0., 1., 0., 0., 0., 0., 0.],
                [0., 0., 0., 0., 0., 0., 1., 0., 0.]])
f.R = 5

def kinematic_predict(delt):
    f.F = np.array([[1.,    delt,   0.5*(delt**2),     0., 0., 0., 0., 0., 0.],
                [0.,    1.,     delt,               0., 0., 0., 0., 0., 0.],
                [0.,    0.,     1.,                 0., 0., 0., 0., 0., 0.],
                [0., 0., 0.,   1.,    delt,   0.5*(delt**2),   0., 0., 0.],
                [0., 0., 0.,    0.,    1.,     delt,            0., 0., 0.],
                [0., 0., 0.,    0.,    0.,     1.,              0., 0., 0.],
                [0., 0., 0., 0., 0., 0.,   1.,    delt,   0.5*(delt**2)],
                [0., 0., 0., 0., 0., 0.,    0.,    1.,     delt],
                [0., 0., 0., 0., 0., 0.,    0.,    0.,     1.]])
    f.P *= 1000.
    f.Q = Q_discrete_white_noise(dim=3, dt=delt, block_size = 3, var=0.13, order_by_dim = True)
    f.predict()
    return ([f.x[0], f.x[3], f.x[6]])

def kinematic_Update(x, y, z):
    f.update([x,y,z])

# i=0
# while (i<20):
#     x = ap[i]
#     y = aq[i]
#     z = ar[i]
#     f.predict()
#     print("(", x, ",", f.x[0], ")", "\t\t(", y, ",", f.x[3], ")", "\t\t(", z, ",", f.x[6], ")")
#     f.update([x,y,z])
#     i += 1