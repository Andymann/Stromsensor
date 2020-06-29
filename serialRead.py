#!/usr/bin/env python
import time
import serial
import os
from datetime import datetime
from time import sleep

def openPort():
    global ser
    ser = serial.Serial(
        port='/dev/ttyUSB0',
        baudrate = 9600,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        bytesize=serial.EIGHTBITS,
        timeout=90
    )

while 1:
    openPort()
    now = datetime.now()
    current_time = now.strftime("%H:%M:%S")
    x=ser.readline()
    ser.flush()
    ser.close()
    print( current_time + '   ' + str(x.strip()) )
    file1 = open("/home/pi/Documents/datalog.txt","a")
    file1.write( current_time + '   ' + str(x.strip()) + os.linesep )
    file1.close()
    sleep(30)
