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
f.write("*******************\n")
while True:
    queue = ser.inWaiting()

    if queue > 0:
        sleep(0.2)
        data = ser.read(18)
        print(data)
        f.write(data)
        data_count = data_count + 1

        if data_count == 19:
            data_count = 1
            log_count = log_count + 1

            f.write("\nLog %d\n" %log_count)
            f.write("*****\n")

    sleep(0.2)

f.close
