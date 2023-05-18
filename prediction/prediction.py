from filterpy.kalman import UnscentedKalmanFilter
from filterpy.kalman import MerweScaledSigmaPoints
import numpy as np
from filterpy.common import Q_discrete_white_noise

#Current Prediction class with Unscented Kalman Filters
#Do not try to rename the self.filter variables

class Prediction(object):    
    def __init__(self):
        #Generates the dot product of the state transition matrix and an input matrix x
        def stateDotX(inputMatrix, delt):
            stateTransitionMatrix = np.array([[1.,  delt,   0.5*(delt**2),      (1/6)*(delt**3),0., 0., 0., 0., 0., 0., 0., 0.],
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
            return np.dot(stateTransitionMatrix, inputMatrix)

        #Returns the current x position, y position, and z position
        def currentPos(inputMatrix):
            return np.array([inputMatrix[0], inputMatrix[4], inputMatrix[8]])

        #Generates needed sigma points
        points = MerweScaledSigmaPoints(12, alpha=.1, beta=2., kappa=-1)

        # dim_x - number of Kalman filter state variables (position, velocity, acceleration in x, y, z directions = 9)
        #         also includes jerk (j) in x, y, z directions
        # dim_z - number of measurement inputs (x, y, z = 3)
        self.filter = UnscentedKalmanFilter(dim_x=12, dim_z=3, dt=0.1, fx=stateDotX, hx=currentPos, points=points)


        # X - track current estimates of x, x', x'', x''', y, y', y'', y''', z, z', z'', z'''
        self.filter.x = np.array([0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.])

        # H - observation matrix (makes sizes consistent)
        self.filter.H = np.array([  [1., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0., 0.],
                                    [0., 0., 0., 0., 1., 0., 0., 0., 0., 0., 0., 0.],
                                    [0., 0., 0., 0., 0., 0., 0., 0., 1., 0., 0., 0.]])
        
        # R - measurement noise covariance matrix
        self.filter.R *= .5

        #Covariance Matrix
        self.filter.P *= 10

    #Called whenever a n+1 position is needed
    def kinematicPredict(self, delt):
        #State Transition Matrix with x,v,a,j
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
        
        #Noise Matrix
        self.filter.Q = Q_discrete_white_noise(dim=4, dt=delt, block_size = 3, var=0.13, order_by_dim = True)

        #Solves for a N+1 solution and places it in filter.x
        self.filter.predict(dt=delt)
        
    #Returns x', y', z', x'', y'', z''
    def getVA(self):
        return [self.filter.x[1], self.filter.x[2], self.filter.x[5], self.filter.x[6], self.filter.x[9], self.filter.x[10]]

    #Returns x, y, z
    def getPredictedPos(self):
        return[self.filter.x[0], self.filter.x[4], self.filter.x[8]]

    #Updates the weights in the Unscented Kalman Filter
    def kinematicUpdate(self, pos):
        self.filter.update(pos)