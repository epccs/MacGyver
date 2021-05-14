#!/usr/bin/env python3
# The above line tells the command shell to run env which will then run Python3.
# Unfortunately when I edit this file on Windows it saves with a CR-LF line-ending
# which will confuse the command processor. Line ending is the gift from hell.
# Anyway make sure that top line ends with a LF only. Python will work with a CR-LF or LF line ending 
# but that first line may confuse the OS runing on Linux and fail. 
# This python file also needs to be set as an executable with chmod +x <this_file>

# Pyserial has got worked on since last I used it, and I want to try the new stuff
# $ sudo apt install python3-pip
# $ pip3 install pyserial
#    Collecting pyserial
#    Downloading pyserial-3.3-py2.py3-none-any.whl (189kB)

import serial, struct, time
ser = serial.Serial('/dev/ttyUSB1',38400, timeout=3)

# we only need to open the port since that will reset the Atmega328pb, which starts in UART mode.

while ser.in_waiting > 0: 
    bootmsg = ser.readline().strip() # report any bootmsg
    print("bootmsg: " + bootmsg.decode("utf-8"))
    
cmd = '/0/uart\n' # the command that will tell RPUusb board to simulate R-Pi BCM23=HIGH: BCM24=LOW for UART mode
#  if we needed that, but since RPUusb has been rebooted the UPDImode firmware (included in its i2c-debug) is back in UART mode.


