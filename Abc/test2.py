from filterpy.kalman import KalmanFilter
import numpy as np
from filterpy.common import Q_discrete_white_noise
import test
#30 samples
ap = np.array([6, 28, 86, 180, 310, 476, 678, 916, 1190, 1500, 1846, 2228, 2646, 3100, 3590, 4116, 4678, 5276, 5910, 6580, 7286, 8028, 8806, 9620, 10470, 11356, 12278, 13236, 14230, 15260])
aq = np.array([22, 35, 54, 79, 110, 147, 190, 239, 294, 355, 422, 495, 574, 659, 750, 847, 950, 1059, 1174, 1295, 1422, 1555, 1694, 1839, 1990, 2147, 2310, 2479, 2654, 2835])
ar = np.array([8, 18, 46, 92, 156, 238, 338, 456, 592, 746, 918, 1108, 1316, 1542, 1786, 2048, 2328, 2626, 2942, 3276, 3628, 3998, 4386, 4792, 5216, 5658, 6118, 6596, 7092, 7606])
sp = 0.1
m = 30
xm = ap + sp * (np.random.randn(m))
ym = aq + sp * (np.random.randn(m))
zm = ar + sp * (np.random.randn(m))
delt = 0.1
f = test.Prediction()

i=0
while (i<30):
    f.kinematicPredict(delt)
    pos = f.getPredictedPos()
    print("({:.2f}".format(pos[0]), ", {:.2f})".format(xm[i]), "\t\t({:.2f}".format(pos[1]), ", {:.2f})".format(ym[i]), "\t\t({:.2f}".format(pos[2]), ", {:.2f})".format(zm[i]))
    f.kinematicUpdate(ap[i], aq[i], ar[i])
    i += 1