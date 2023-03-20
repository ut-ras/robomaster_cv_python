from filterpy.kalman import KalmanFilter
import matplotlib.pyplot as plt
import numpy as np
import array as arr
from filterpy.common import Q_discrete_white_noise

dt = 0.1
ap = np.array([1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20])
aq = np.array([20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1])
f = KalmanFilter (dim_x=3, dim_z=1)
g = KalmanFilter (dim_x=3, dim_z=1)
f.x = np.array([100., 10., 1.])
f.F = np.array([[1., dt, 0.5*(dt**2)],
                [0., 1., dt],
                [0., 0., 1.]])
f.H = np.array([[1., 0., 0.]])
f.P *= 1000.
f.R = 5
f.Q = Q_discrete_white_noise(dim=3, dt=0.1, var=0.13)
g.x = np.array([100., 10., 1.])
g.F = np.array([[1., dt, 0.5*(dt**2)],
                [0., 1., dt],
                [0., 0., 1.]])
g.H = np.array([[1., 0., 0.]])
g.P *= 1000.
g.R = 5
g.Q = Q_discrete_white_noise(dim=3, dt=0.1, var=0.13)
i = 0
while (i < 20):
    x = ap[i]
    y = aq[i]
    f.predict()
    g.predict()
    print("(", x, ",", f.x[0], ")", " (", y, ",", g.x[0], ")")
    f.update(x)
    g.update(y)
    i+=1