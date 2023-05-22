import serial
import ctypes

# Serial communication

ser = serial.Serial()
def initialize_communication():
    ser.port = '/dev/bone/ttyS4'
    ser.baudrate = 115200
    ser.timeout = 0.5
    ser.open()

def send_message(message):
    ser.write(message)

def read_message():
    message = ser.readline()
    return message


# Formatting structs
# ctypes version

class TurretData(ctypes.Structure):
    _fields_ = [("x_Pos", ctypes.c_float),
                ("y_Pos", ctypes.c_float),
                ("z_Pos", ctypes.c_float),
                ("x_Vel", ctypes.c_float),
                ("y_Vel", ctypes.c_float),
                ("z_Vel", ctypes.c_float),
                ("x_Acc", ctypes.c_float),
                ("y_Acc", ctypes.c_float),
                ("z_Acc", ctypes.c_float),
                ("has_Target", ctypes.c_bool)]

def format_turret(xPos, yPos, zPos, 
                  xVel, yVel, zVel, 
                  xAcc, yAcc, zAcc, 
                  hasTarget):
    tur = TurretData(xPos, yPos, zPos, 
                  xVel, yVel, zVel, 
                  xAcc, yAcc, zAcc, 
                  hasTarget)