import serial
import struct
import crc
import sys
from enum import IntEnum

"""
Communication.py is aimed to send information to the beaglebone. The way of doing so is to utilize UART
"""

# Serial communication
ser = serial.Serial()

"""
Initialize_communication initializes the communication port. Tentatively, the COM port is /dev/bone/ttyS4 with a baud rate of 115200 and a timeout of 0.5
"""
def initialize_communication():
    ser.port = '/dev/ttyS4'
    ser.baudrate = 115200
    ser.timeout = 0.5
    ser.open()


# https://docs.python.org/3/library/struct.html#format-strings
TurretDataFormat = struct.Struct("<fffffffff?")
FrameHeaderFormat = struct.Struct("<BHB")

def read_message():
    message = ser.readline()
    return message

def send_message(data, data_format, message_type):
    frame_head_byte = 0xA5
    frame_data_length = data_format.size
    frame_sequence_number = 0

    frame_header = FrameHeaderFormat.pack(frame_head_byte, frame_data_length, frame_sequence_number)
    frame_crc = crc8.checksum(frame_header)
    
    MessageNoCRC16Format = struct.Struct('<{}sBH{}s'.format(FrameHeaderFormat.size, frame_data_length))
    message_no_crc16 = MessageNoCRC16Format.pack(frame_header, frame_crc, message_type.value, data)
    message_crc = crc16.checksum(message_no_crc16)
    
    FullMessageFormat = struct.Struct('<{}sH'.format(MessageNoCRC16Format.size))
    full_message = FullMessageFormat.pack(message_no_crc16, message_crc)

    ser.write(full_message)


crc8 = crc.Calculator(crc.Crc8.MAXIM_DOW)

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
class MessageType(IntEnum):
    CMD_Odometry_Data = 1
    CMD_Turret_Aim = 2

# Sending message
def send_turret_data(xPos, yPos, zPos, 
                  xVel, yVel, zVel, 
                  xAcc, yAcc, zAcc, 
                  hasTarget):
    send_message(TurretDataFormat.pack(xPos, yPos, zPos, xVel, yVel, zVel, xAcc, yAcc, zAcc, hasTarget), TurretDataFormat, MessageType.CMD_Turret_Aim)