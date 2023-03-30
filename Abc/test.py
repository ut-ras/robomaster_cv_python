from filterpy.kalman import KalmanFilter
import numpy as np
from filterpy.common import Q_discrete_white_noise

class Prediction(object):

    def __init__(self):
        # dim_x - number of Kalman filter state variables (position, velocity, acceleration in x, y, z directions = 9)
        # dim_z - number of measurement inputs (x, y, z = 3)
        self.filter = KalmanFilter(dim_x=9, dim_z=3)
        # X - track current estimates of x, x', x'', y, y', y'', z, z', z''
        self.filter.x = np.array([0., 0., 0., 0., 0., 0., 0., 0., 0.])
        # H - observation matrix (makes sizes consistent)
        self.filter.H = np.array([[1., 0., 0., 0., 0., 0., 0., 0., 0.],
                        [0., 0., 0., 1., 0., 0., 0., 0., 0.],
                        [0., 0., 0., 0., 0., 0., 1., 0., 0.]])
        # R - measurement noise covariance matrix
        self.filter.R = 5.

    # 
    def kinematicPredict(self, delt):
        pass
        self.filter.F = np.array([[1.,    delt,   0.5*(delt**2),     0., 0., 0., 0., 0., 0.],
                    [0.,    1.,     delt,               0., 0., 0., 0., 0., 0.],
                    [0.,    0.,     1.,                 0., 0., 0., 0., 0., 0.],
                    [0., 0., 0.,   1.,    delt,   0.5*(delt**2),   0., 0., 0.],
                    [0., 0., 0.,    0.,    1.,     delt,            0., 0., 0.],
                    [0., 0., 0.,    0.,    0.,     1.,              0., 0., 0.],
                    [0., 0., 0., 0., 0., 0.,   1.,    delt,   0.5*(delt**2)],
                    [0., 0., 0., 0., 0., 0.,    0.,    1.,     delt],
                    [0., 0., 0., 0., 0., 0.,    0.,    0.,     1.]])
        self.filter.P *= 1000.
        self.filter.Q = Q_discrete_white_noise(dim=3, dt=delt, block_size = 3, var=0.13, order_by_dim = True)
        self.filter.predict()
        
    #x', y', z', x'', y'', z''
    def getVA(self):
        return [self.filter.x[1], self.filter.x[4], self.filter.x[7], self.filter.x[2], self.filter.x[5], self.filter.x[8]]

    #x, y, z
    def getPredictedPos(self):
        return[self.filter.x[0], self.filter.x[3], self.filter.x[6]]

    def kinematicUpdate(self, x, y, z):
        self.filter.update([x,y,z])