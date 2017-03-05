import serial, sys, time, io

from time import sleep

todays_date = time.strftime("%c")
f = open(todays_date, "w+")

try:
    ser = serial.Serial("/dev/cu.usbserial-A603H9LP", 9600,timeout=0, parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
except:
    sys.exit("Error connecting device")

log_count = 1
data_count = 1

f.write("Log 1\n")
f.write("*****\n")
while True:
    queue = ser.inWaiting()

    if queue > 0:
        if data_count == 1:
            f.write("Date: ")
        if data_count == 3:
            f.write("h")
        if data_count == 5:
            f.write(" ")
        if data_count == 7:
            f.write("/")
        if data_count == 9:
            f.write("/")
        if data_count == 9:
            f.write("/")
        if data_count == 11:
            f.write("\n Execution time")
        if data_count == 14:
            f.write("s Eska with a Cap: ")
        if data_count == 15:
            f.write("Eska without a Cap:")
        if data_count == 16:
            f.write("Yop with a Cap:")
        if data_count == 17:
            f.write("Yop without a Cap:")
        if data_count == 18:
            f.write("Stop condition:")

        data = ser.read(1)
        print(data)
        f.write(data)
        data_count = data_count + 1

    sleep(0.2)

f.close
