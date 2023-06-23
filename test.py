from communication import communication as com
import numpy as np
com.initialize_communication()
com.send_turret_data(xPos = np.float32(1), yPos = np.float32(2), zPos = np.float32(3),
xVel = np.float32(4),yVel = np.float32(5), zVel = np.float32(6),
xAcc = np.float32(7), yAcc = np.float32(8), zAcc = np.float32(9),
hasTarget=True)