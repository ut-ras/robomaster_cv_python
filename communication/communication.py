import serial

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


