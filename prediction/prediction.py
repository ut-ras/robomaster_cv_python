from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints
import numpy as np
from filterpy.common import Q_discrete_white_noise

class Prediction(object):  
    def __init__(self):
        # returns the state matrix x transformed by the state transition function
        def fx(x, delt):
            F = np.array([[1., delt, 0.5*(delt**2), (1/6)*(delt**3), 0.,   0.,            0.,              0., 0.,   0.,            0.,              0.],
                          [0.,   1.,          delt,   0.5*(delt**2), 0.,   0.,            0.,              0., 0.,   0.,            0.,              0.],
                          [0.,   0.,            1.,            delt, 0.,   0.,            0.,              0., 0.,   0.,            0.,              0.],
                          [0.,   0.,            0.,              1., 0.,   0.,            0.,              0., 0.,   0.,            0.,              0.],
                          [0.,   0.,            0.,              0., 1., delt, 0.5*(delt**2), (1/6)*(delt**3), 0.,   0.,            0.,              0.],
                          [0.,   0.,            0.,              0., 0.,   1.,          delt,   0.5*(delt**2), 0.,   0.,            0.,              0.],
                          [0.,   0.,            0.,              0., 0.,   0.,            1.,            delt, 0.,   0.,            0.,              0.],
                          [0.,   0.,            0.,              0., 0.,   0.,            0.,              1., 0.,   0.,            0.,              0.],
                          [0.,   0.,            0.,              0., 0.,   0.,            0.,              0., 1., delt, 0.5*(delt**2), (1/6)*(delt**3)],
                          [0.,   0.,            0.,              0., 0.,   0.,            0.,              0., 0.,    1.,         delt,   0.5*(delt**2)],
                          [0.,   0.,            0.,              0., 0.,   0.,            0.,              0., 0.,    0.,           1.,            delt],
                          [0.,   0.,            0.,              0., 0.,   0.,            0.,              0., 0.,    0.,           0.,              1.]])
            return np.dot(F, x)

        # gets (x, y, z) position measurements from state matrix x
        def hx(x):
            return np.array([x[0], x[4], x[8]])
    
        points = MerweScaledSigmaPoints(12, alpha=1e-3, beta=2., kappa=-9)
        # dim_x - number of Kalman filter state variables (position, velocity, acceleration, jerk in x, y, z directions = 12)
        # dim_z - number of measurement inputs (x, y, z = 3)
        # dt is overridden ever time we make a prediction, so 0.1 value is arbitrary
        self.filter = UnscentedKalmanFilter(dim_x=12, dim_z=3, dt=0.1, fx=fx, hx=hx, points=points)
        # x - state matrix to track current estimates of x, x', x'', x''', y, y', y'', y''', z, z', z'', z'''
        self.filter.x = np.array([0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])
        # H - observation matrix (makes sizes consistent)
        self.filter.H = np.array([[1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                  [0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.],
                                  [0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.]])
        # R - measurement noise covariance matrix
        self.filter.R *= .5
        # P - Covariance Matrix
        self.filter.P *= 10

    # usage note - MUST call kinematicPredict() at least once before calling kinematicUpdate()
    def kinematicPredict(self, delt):
        # update state transition matrix with based on delt
        self.filter.F = np.array([[1.,  delt,   0.5*(delt**2),      (1/6)*(delt**3),0., 0., 0., 0., 0., 0., 0., 0.],
                                [0.,    1.,     delt,               0.5*(delt**2),  0., 0., 0., 0., 0., 0., 0., 0.],
                                [0.,    0.,     1.,                 delt,           0., 0., 0., 0., 0., 0., 0., 0.],
                                [0.,    0.,     0.,                 1.,             0., 0., 0., 0., 0., 0., 0., 0.],
                                [0., 0., 0., 0.,    1., delt,   0.5*(delt**2),  (1/6)*(delt**3),    0., 0., 0., 0.],
                                [0., 0., 0., 0.,    0., 1.,     delt,           0.5*(delt**2),      0., 0., 0., 0.],
                                [0., 0., 0., 0.,    0., 0.,     1.,             delt,               0., 0., 0., 0.],
                                [0., 0., 0., 0.,    0., 0.,     0.,             1.,               0., 0., 0., 0.],
                                [0., 0., 0., 0., 0., 0., 0., 0,   1.,    delt,   0.5*(delt**2), (1/6)*(delt**3)],
                                [0., 0., 0., 0., 0., 0., 0., 0,   0.,    1.,     delt,          0.5*(delt**2)],
                                [0., 0., 0., 0., 0., 0., 0., 0,   0.,    0.,     1.,            delt],
                                [0., 0., 0., 0., 0., 0., 0., 0,   0.,    0.,     0.,            1.]])
        # Q - noise matrix
        self.filter.Q = Q_discrete_white_noise(dim=4, dt=delt, block_size=3, var=0.13, order_by_dim=True)
        # solves for a N+1 solution and places it in filter.x
        self.filter.predict(dt=delt)
        
    # [x', x'', y', y'', z', z'']
    def getVA(self):
        return [self.filter.x[1], self.filter.x[2], self.filter.x[5], self.filter.x[6], self.filter.x[9], self.filter.x[10]]

    # [x, y, z]
    def getPredictedPos(self):
        return [self.filter.x[0], self.filter.x[4], self.filter.x[8]]

    # usage note - MUST call kinematicPredict() at least once before calling kinematicUpdate()
    def kinematicUpdate(self, pos):
        self.filter.update(pos)

    # def getLikelihood(self):
    #     return self.filter.likelihood()