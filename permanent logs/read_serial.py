import serial, sys, time, io

from time import sleep

todays_date = time.strftime("%c")
f = open(todays_date, "w+")

f.write("botL\n")
f.write(todays_date)
f.write("\n\n")

f.write("Guide: \n")
f.write("18 bits of data are saved for every run. this log will display data for the 9 latest runs. here is the bit breakdown: \n")
f.write("   0-9 are the date (hourhour/minmin/yy/mm/dd)\n")
f.write("   10-12 are the operation time in seconds (100x/10x/1x)\n")
f.write("   13-16 are the sorting statistics (ESKA/ESKANOCAP/YOP/YOPNOCAP)\n")
f.write("   17 is the stop condition (0: Normal, 1: Emergency Stop, 2: Execution Timeout, 3: Optimized Stop)\n\n")

try:
    ser = serial.Serial("/dev/cu.usbserial-A603H9LP", 9600,timeout=0, parity=serial.PARITY_NONE,
                        stopbits=serial.STOPBITS_ONE, bytesize=serial.EIGHTBITS)
except:
     sys.exit("Error connecting device")

log_count = 1
data_count = 1

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
