from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints
import numpy as np
from filterpy.common import Q_discrete_white_noise
import logging
from posdef import nearestPD


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
            x_pos = x[0]
            y_pos = x[4]
            z_pos = x[8]
            return np.array([x_pos, y_pos, z_pos])
        points = MerweScaledSigmaPoints(12, alpha=1e-3, beta=2., kappa=-9, sqrt_method=self.sqrt_func)

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
        self.filter.R *= 0.01
        # P - Covariance Matrix
        self.filter.P *= 0.1 # was 19

    def sqrt_func(self, x):
        
        try:
            result = np.linalg.cholesky(x)
        except np.linalg.LinAlgError:
            try:
                logging.warn("Nth Leading Minor not positive")
                x = (x + x.T)/2
                result = np.linalg.cholesky(x)
            except np.linalg.LinAlgError:
                logging.warn("Nth Leading Minor still not positive")
                x = nearestPD(x)
        finally:
            return result

    # usage note - MUST call kinematicPredict() at least once before calling kinematicUpdate()
    def kinematicPredict(self, del_t):
        # Q - noise matrix
        self.filter.Q = Q_discrete_white_noise(dim=4, dt=del_t, block_size=3, var=0.13, order_by_dim=True)
        # solves for a N+1 solution and places it in filter.x
        self.filter.predict(dt=del_t)
        
    # [x', x'', y', y'', z', z'']
    def getVA(self):
        # x_vel = self.filter.x[1]
        # x_acc = self.filter.x[2]
        # y_vel = self.filter.x[5]
        # y_acc = self.filter.x[6]
        # z_vel = self.filter.x[9]
        # z_acc = self.filter.x[10]

        vel = {
            "x_vel":self.filter.x[1],
            "y_vel":self.filter.x[5],
            "z_vel":self.filter.x[9],
        }

        acc = {
            "x_acc":self.filter.x[2],
            "y_acc":self.filter.x[6],
            "z_acc":self.filter.x[10],
        }
        
        #return [self.filter.x[1], self.filter.x[2], self.filter.x[5], self.filter.x[6], self.filter.x[9], self.filter.x[10]]
        return vel, acc

    # [x, y, z]
    def getPredictedPos(self):

        position = {
            "x_pos": self.filter.x[0],
            "y_pos": self.filter.x[4],
            "z_pos": self.filter.x[8]
        }
        return position

    # usage note - MUST call kinematicPredict() at least once before calling kinematicUpdate()
    #pos - np.ndarray(x,y,z) position
    def kinematicUpdate(self, pos):
        self.filter.update(pos)

    # def getLikelihood(self):
    #     return self.filter.likelihood()