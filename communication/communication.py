import serial
import ctypes
import struct
from enum import Enum


# Serial communication
ser = serial.Serial()
def initialize_communication():
    ser.port = '/dev/bone/ttyS4'
    ser.baudrate = 115200
    ser.timeout = 0.5
    ser.open()

def read_message():
    message = ser.readline()
    return message

def send_message(message):
    ser.write(message)


# Reading message
# Message type
class MessageType(Enum):
    CMD_Odometry_Data = 1
    CMD_Turret_Aim = 2

def find_message:


# Sending message
# Formatting struct

TurretData = struct.Struct("<fffffffff?")

# @params turret data
def format_turret(xPos, yPos, zPos, 
                  xVel, yVel, zVel, 
                  xAcc, yAcc, zAcc, 
                  hasTarget):
    TurretData.pack(xPos, yPos, zPos, 
                  xVel, yVel, zVel, 
                  xAcc, yAcc, zAcc, 
                  hasTarget)