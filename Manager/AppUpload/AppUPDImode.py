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
time.sleep(3.1) # wait for the bootloader to run
while ser.in_waiting > 0: 
    bootmsg = ser.readline().strip() # report any bootmsg
    print("bootmsg: " + bootmsg.decode("utf-8"))

cmd = '/0/iaddr 42\n' # i2c address of manager from the host (SMBus)
# Python 3 strings are utf-8, they seem to need encode to ascii for Pyserial
ser.write(cmd.encode('ascii'))
echo = ser.readline().strip() # read the echo
print("cmd echo: " + echo.decode("utf-8"))
response = ser.readline().strip() # read the response
print("response: " + response.decode("utf-8"))

print("#: Manager/AppUpload firmware looks for a 7 on i2c to enable Application UPID mode")
cmd = '/0/ibuff 7\n' # add a byte with value 7 to i2c buffer
ser.write(cmd.encode('ascii'))
echo = ser.readline().strip() # read the echo
print("cmd echo: " + echo.decode("utf-8"))
response = ser.readline().strip() # read the response
print("response: " + response.decode("utf-8"))

cmd = '/0/iread? 1\n' # send the buffer and read back same number of bytes
ser.write(cmd.encode('ascii'))
echo = ser.readline().strip() # read the echo
print("cmd echo: " + echo.decode("utf-8"))
response = ser.readline().strip() # read the response
print("response: " + response.decode("utf-8"))
