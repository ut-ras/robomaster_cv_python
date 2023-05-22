import serial
import struct
import crc
import sys
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

def send_message(data):
    data_length = sys.getsizeof(data)
    frame_message = FrameHeaderFormat.pack(0xA5, data_length, 0)
    checksum = crc8.checksum(frame_message)

    
    ser.write(data)


crc8 = crc.Calcuator(crc.Crc8.MAXIM_DOW)

crc16config = crc.Configuration(
    width=16,
    polynomial=0x1021,
    init_value=0x0000,
    final_xor_value=0x0000,
    reverse_input=True,
    reverse_output=True,
)

crc16 = crc.Calculator(crc16config)


# Reading message
class MessageType(Enum):
    CMD_Odometry_Data = 1
    CMD_Turret_Aim = 2

# Sending message
def send_turret_data(xPos, yPos, zPos, 
                  xVel, yVel, zVel, 
                  xAcc, yAcc, zAcc, 
                  hasTarget):
    send_message(TurretDataFormat.pack(xPos, yPos, zPos, xVel, yVel, zVel, xAcc, yAcc, zAcc, hasTarget))

TurretDataFormat = struct.Struct("<fffffffff?")
FrameHeaderFormat = struct.Struct("<cHccH")
    