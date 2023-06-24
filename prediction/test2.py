from filterpy.kalman import KalmanFilter
import numpy as np
from filterpy.common import Q_discrete_white_noise
import prediction
import matplotlib.pyplot as plt
import math

SAMPLE_SIZE = 100

def genSinFunction(amp:int,endVal):
    return amp * np.sin(np.linspace(0,endVal,SAMPLE_SIZE))
sample = {
    "sinX": genSinFunction(1000,4*np.pi),
    "sinY": genSinFunction(400,np.pi),
    "sinZ": genSinFunction(200,6*np.pi),
    "sine": genSinFunction(1,2*np.pi)
}


"""
testKalman filter takes three arrays of values in x, y, and z which indicates position over time
"""
def testKalmanFilter(xValues,yValues,zValues):
    #variance of randomness
    m = 0.1
    #Noisy Data
    xm = xValues + np.random.normal(0,abs(m*xValues))
    ym = yValues + np.random.normal(0,abs(m*yValues))
    zm = zValues + np.random.normal(0,abs(m*zValues))
    #Time Step
    delt = 1/30     #assumes 30 fps
    #Create Prediction Object
    f = prediction.Prediction()

    #Iteration Counter
    predVals_x = []
    predVals_y = []
    predVals_z = []

    i=0
    while (i<SAMPLE_SIZE):
        #Predicts N+1 time
        f.kinematicPredict(delt)
        #Retrieves Postion
        pos = f.getPredictedPos()
        #Prints Position
        print("({:.2f}".format(pos["x_pos"]), ", {:.2f})".format(xm[i]), "\t\t({:.2f}".format(pos["y_pos"]), ", {:.2f})".format(ym[i]), "\t\t({:.2f}".format(pos["z_pos"]), ", {:.2f})".format(zm[i]))
        # f.getLikelihood()
        predVals_x.append(pos["x_pos"])
        predVals_y.append(pos["y_pos"])
        predVals_z.append(pos["z_pos"])
        

        #Updates matricies with new noisy measurement
        f.kinematicUpdate([xm[i], ym[i], zm[i]])
        #Iteration
        i += 1
    t = [i for i in range(0,SAMPLE_SIZE)]

    fig1 = plt.figure("X-values")
    plt.plot(t, xValues, label = 'noiseless data')
    plt.plot(t,predVals_x, label = 'predicted')
    plt.plot(t,xm, label = 'noisy data')
    plt.legend()
    plt.xlabel("timesteps (assuming 30 fps)")
    plt.ylabel("position")
    plt.title("X-values predicted versus actual data")


    fig2 = plt.figure("Y-values")
    plt.plot(t, yValues, label = 'noiseless data')
    plt.plot(t,predVals_y, label = 'predicted')
    plt.plot(t,ym, label = 'noisy data')
    plt.title("Y-values predicted versus actual data")
    plt.legend()
    plt.xlabel("timesteps (assuming 30 fps)")
    plt.ylabel("position")

    fig3 = plt.figure("Z-values")
    plt.plot(t, zValues, label = 'noiseless data')
    plt.plot(t,predVals_z, label = 'predicted')
    plt.plot(t,zm, label = 'noisy data')
    plt.xlabel("timesteps (assuming 30 fps)")
    plt.ylabel("position")
    plt.title("Z-values predicted versus actual data")
    plt.legend()
    plt.show()


testKalmanFilter(sample['sinX'],sample["sinY"],sample["sinZ"])