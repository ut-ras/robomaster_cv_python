import serial
import struct
import crc
import sys
from enum import IntEnum
import logging
import time

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
    ser.open()

color_data_size = 20

#  * Structure of a Serial Message:
#  * \rst
#  * +-----------------+------------------------------------------------------------+
#  * | Byte Number     | Byte Description                                           |
#  * +=================+============================================================+
#  * | Frame Header                                                                 |
#  * +-----------------+------------------------------------------------------------+
#  * | 0               | Frame Head Byte (0xA5)                                     |
#  * +-----------------+------------------------------------------------------------+
#  * | 1               | Frame Data Length, LSB                                     |
#  * +-----------------+------------------------------------------------------------+
#  * | 2               | Frame Data Length, MSB                                     |
#  * +-----------------+------------------------------------------------------------+
#  * | 3               | Frame Sequence Number                                      |
#  * +-----------------+------------------------------------------------------------+
#  * | 4               | CRC8 of the frame, (bytes 0 - 3)                           |
#  * +-----------------+------------------------------------------------------------+
#  * | 5               | Message Type, LSB                                          |
#  * +-----------------+------------------------------------------------------------+
#  * | 6               | Message Type, MSB                                          |
#  * +-----------------+------------------------------------------------------------+
#  * | Body - Data Length bytes                                                     |
#  * +-----------------+------------------------------------------------------------+
#  * | Message CRC                                                                  |
#  * +-----------------+------------------------------------------------------------+
#  * | 7 + Data Length | CRC16 of header and frame, LSB (bytes 0 - 6 + Data Length) |
#  * +-----------------+------------------------------------------------------------+
#  * | 8 + Data Length | CRC16 of header and frame, MSB                             |
#  * +-----------------+------------------------------------------------------------+
#  * \endrst
#  */
def read_message():
    message = ser.read(color_data_size)

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

# https://docs.python.org/3/library/struct.html#format-strings
TurretDataFormat = struct.Struct("<fffffffff?")
FrameHeaderFormat = struct.Struct("<BHB")
ColorDataFormat = struct.Struct("<BHB")

crc8config = crc.Configuration(
    width=8,
    polynomial=0x31,
    init_value=0xFF,
    final_xor_value=0x00,
    reverse_input=True,
    reverse_output=True,
)

crc16config = crc.Configuration(
    width=16,
    polynomial=0x1021,
    init_value=0xFFFF,
    final_xor_value=0x0000,
    reverse_input=True,
    reverse_output=True,
)

crc8 = crc.Calculator(crc8config)

crc16 = crc.Calculator(crc16config)


# Reading message
class MessageType(IntEnum):
    CMD_Odometry_Data = 1
    CMD_Turret_Aim = 2

# Sending message
def send_turret_data(pos, vel, acc, 
                  hasTarget):
    xPos = pos['x_pos']
    yPos = pos['y_pos']
    zPos = pos['z_pos']
    xVel = vel['x_vel']
    yVel = vel['y_vel']
    zVel = vel['z_vel']
    xAcc = acc['x_acc']
    yAcc = acc['y_acc']
    zAcc = acc['z_acc']
    send_message(TurretDataFormat.pack(xPos, yPos, zPos, xVel, yVel, zVel, xAcc, yAcc, zAcc, hasTarget), TurretDataFormat, MessageType.CMD_Turret_Aim)

def send_no_data():
    yPos = 0.0
    xPos = 0.0
    zPos = 0.0
    xVel = 0.0
    yVel = 0.0
    zVel = 0.0
    xAcc = 0.0
    yAcc = 0.0
    zAcc = 0.0
    hasTarget = False
    send_message(TurretDataFormat.pack(xPos, yPos, zPos, xVel, yVel, zVel, xAcc, yAcc, zAcc, hasTarget), TurretDataFormat, MessageType.CMD_Turret_Aim)
