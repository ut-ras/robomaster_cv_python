from filterpy.kalman import KalmanFilter
import numpy as np
from filterpy.common import Q_discrete_white_noise
import prediction
import matplotlib.pyplot as plt
#30 samples
ap = np.array([6, 28, 86, 180, 310, 476, 678, 916, 1190, 1500, 1846, 2228, 2646, 3100, 3590, 4116, 4678, 5276, 5910, 6580, 7286, 8028, 8806, 9620, 10470, 11356, 12278, 13236, 14230, 15260])
aq = np.array([22, 35, 54, 79, 110, 147, 190, 239, 294, 355, 422, 495, 574, 659, 750, 847, 950, 1059, 1174, 1295, 1422, 1555, 1694, 1839, 1990, 2147, 2310, 2479, 2654, 2835])
ar = np.array([8, 18, 46, 92, 156, 238, 338, 456, 592, 746, 918, 1108, 1316, 1542, 1786, 2048, 2328, 2626, 2942, 3276, 3628, 3998, 4386, 4792, 5216, 5658, 6118, 6596, 7092, 7606])




#variance of randomness
m = 0.1
#Noisy Data
xm = (np.random.normal(ap,m*ap))
ym = (np.random.normal(aq,m*aq))
zm = (np.random.normal(ar,m*ar))
#Time Step
delt = 0.1
#Create Prediction Object
f = prediction.Prediction()

#Iteration Counter
predVals_x = []
predVals_y = []
predVals_z = []
actualVals_x = []
actualVals_y = []
actualVals_z = []

i=0
while (i<30):
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
    f.kinematicUpdate([ap[i], aq[i], ar[i]])
    #Iteration
    i += 1
t = [i for i in range(0,30)]

fig1 = plt.figure("X-values")
plt.plot(t,predVals_x)
plt.plot(t,xm)
plt.title("X-values predicted versus actual data")


fig2 = plt.figure("Y-values")
plt.plot(t,predVals_y)
plt.plot(t,ym)
plt.title("Y-values predicted versus actual data")

fig3 = plt.figure("Z-values")
plt.plot(t,predVals_z)
plt.plot(t,zm)
plt.title("Z-values predicted versus actual data")
plt.show()