﻿#!/usr/bin/python
# -*- coding: utf-8 -*-
import serial
ser = serial.Serial('/dev/ttyACM1', 9600,timeout=1);   #open named port at 9600,1s timeot

#try and exceptstructure are exception handler
try:
  while 1:
    ser.write('s');#writ a string to port
   response = ser.readall();#read a string from port
   print response;
except:
  ser.close();