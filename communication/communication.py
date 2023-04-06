import serial

serial = serial.Serial('dev/ttyS0')
print(serial.name)
serial.write(b'Hello')
serial.close()

