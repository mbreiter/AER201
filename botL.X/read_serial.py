import time
import serial

ser = serial.Serial('/dev/cu.usbserial-A603H9LP', 9600)

# ser.flushInput()
# ser.flushOutput()

while True:
    bytesToRead = ser.inWaiting()
    data_raw = ser.read(bytesToRead)
    print(data_raw)
