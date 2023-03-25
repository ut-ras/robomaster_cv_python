from filterpy.kalman import KalmanFilter
import numpy as np
from filterpy.common import Q_discrete_white_noise

f = KalmanFilter (dim_x=9, dim_z=3)
#x, x', x'', y, y', y'', z, z', z''
f.x = np.array([0., 0., 0., 0., 0., 0., 0., 0., 0.])
f.H = np.array([[1., 0., 0., 0., 0., 0., 0., 0., 0.],
                [0., 0., 0., 1., 0., 0., 0., 0., 0.],
                [0., 0., 0., 0., 0., 0., 1., 0., 0.]])
f.R = 5.
def kinematic_predict(self, delt):
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
    
#x', y', z', x'', y'', z''
def getVA(self):
    return [f.x[1], f.x[4], f.x[7], f.x[2], f.x[5], f.x[8]]

#x, y, z
def getPredictedPos(self):
    return[f.x[0], f.x[3], f.x[6]]

def kinematic_Update(self, x, y, z):
    f.update([x,y,z])