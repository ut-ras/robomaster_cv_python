from filterpy.kalman import KalmanFilter
import matplotlib.pyplot as plt
import numpy as np
import array as arr
import time
from filterpy.common import Q_discrete_white_noise
timePast = 0
#ap = np.array([0.0, 0.3, 1.2, 2.6999999999999997, 4.8, 7.5, 10.799999999999999, 14.7, 19.2, 24.3, 30.0, 36.3, 43.199999999999996, 50.699999999999996, 58.8, 67.5, 76.8, 86.7, 97.2, 108.3])
#aq = np.array([0.0, 0.8, 3.2, 7.2, 12.8, 20.0, 28.8, 39.2, 51.2, 64.8, 80.0, 96.80000000000001, 115.2, 135.20000000000002, 156.8, 180.0, 204.8, 231.20000000000002, 259.2, 288.8])
delt = 0.1
f = KalmanFilter (dim_x=3, dim_z=1)
f.x = np.array([100., 10., 1.])
f.H = np.array([[1., 0., 0.]])
f.R = 5

g = KalmanFilter (dim_x=3, dim_z=1)
g.x = np.array([100., 10., 1.])
g.H = np.array([[1., 0., 0.]])
g.R = 5

def kinematic_Predict(delt):
    f.F = np.array([[1., delt, 0.5*(delt**2)],
                    [0., 1., delt],
                    [0., 0., 1.]])
    f.P *= 1000.
    f.Q = Q_discrete_white_noise(dim=3, dt=delt, var=0.13)
    g.F = np.array([[1., delt, 0.5*(delt**2)],
                    [0., 1., delt],
                    [0., 0., 1.]])
    g.P *= 1000.
    g.Q = Q_discrete_white_noise(dim=3, dt=delt, var=0.13)
    return np.array([f.predict(), g.predict()])

def kinematic_Update(x, y):
    f.update(x)
    g.update(y)
# i = 0
# while (i < 20):
#     x = ap[i]
#     y = aq[i]
#     f.predict()
#     g.predict()
#     print("(", x, ",", f.x[0], ")", " (", y, ",", g.x[0], ")")
#     f.update(x)
#     g.update(y)
#     i+=1