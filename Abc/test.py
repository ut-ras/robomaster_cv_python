from filterpy.kalman import KalmanFilter
import matplotlib.pyplot as plt
import numpy as np
import array as arr
ap = arr.array('i', [1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20])
f = KalmanFilter (dim_x=3, dim_z=1)
f.x = np.array([100., 10.])
f.F = np.array([[1.,1.],
                [0.,1.]])
f.H = np.array([[1.,0.]])
f.P *= 1000.
f.R = 5
from filterpy.common import Q_discrete_white_noise
f.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.13)
i = 0
while (i < 20):
    z = ap[i]
    f.predict()
    f.update(z)
    print("(", ap[i], ",", f.x[0], ")")
    i+=1