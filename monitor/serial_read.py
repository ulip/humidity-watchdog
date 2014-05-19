#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
import serial
# import serial.tools.list_ports

ser = serial.Serial('/dev/ttyUSB0', baudrate=9600, timeout=4)
curTime = time.localtime()
filename = 'climate_' \
        + curTime.tm_year + '-' \
        + curTime.tm_mon + '-' \
        + curTime.tm_mday + '.log'
outfile = file(filename, 'a')

last_line = ''
buf = ''

while True:
    ser.write('meas\n')
    buf += ser.read(ser.inWaiting())
    if '\n' in buf:
        last_line, buf = buf.split('\n')[-2:]
        curTime = time.localtime()
        date = '{} {} {}\t'.format(curTime.tm_year, \
                                    curTime.tm_mon, \
                                    curTime.tm_mday)
        hour = '{:8.5f}\t'.format(curTime.tm_hour \
                                + curTime.tm_min/60.0 \
                                + curTime.tm_sec/3600.0)
        print date + hour + last_line.strip()
        if day < curTime.tm_mday:
            outfile.close()
            filename = 'climate_' \
                    + curTime.tm_year + '-' \
                    + curTime.tm_mon + '-' \
                    + curTime.tm_mday + '.log'
            outfile = file(filename, 'a')
            day = curTime.tm_mday
        outfile.write(date + hour + last_line + '\n')
    time.sleep(1)

ser.close()
