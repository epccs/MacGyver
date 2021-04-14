# BlinkLED

## ToDo

The UPDImode.py script needs some attention, the output is out of sync.

## Overview

TBD

## Firmware Upload

Uses a UPDI upload tool. Run 'make' to compile and 'make updi' to upload.

```bash
sudo apt-get install make git gcc-avr binutils-avr gdb-avr avr-libc python3-pip
pip3 install pymcuprog
# I was using pyupdi, but it does not seem to support AVR128DB's so I will start using pymcuprog
# pip3 install pyserial intelhex pylint https://github.com/mraardvark/pyupdi/archive/master.zip
git clone https://github.com/epccs/MacGyver/
# RPUusb has some programs to control UPDI and UART modes
git clone https://github.com/epccs/RPUusb
cd /MacGyver/Manager/BlinkLED
make all
...
make updi
testing for prerequesetits a false will stop make
which python3 2>/dev/null || false
/usr/bin/python3
which pymcuprog 2>/dev/null || false
/home/rsutherland/.local/bin/pymcuprog
ls ../../../RPUusb/UPDImode/UPDImode.py 2>/dev/null || false
../../../RPUusb/UPDImode/UPDImode.py
ls ../../../RPUusb/UPDImode/UARTmode.py 2>/dev/null || false
../../../RPUusb/UPDImode/UARTmode.py
python3 ../../../RPUusb/UPDImode/UPDImode.py
junk: ds include
cmd echo: /0/id?
response: /0/updi
pymcuprog write -t uart -u /dev/ttyUSB0 -d avr128db32 --verify -f BlinkLED.hex
Writing from hex file...
Writing flash...
Verifying flash...
OK
Done.
python3 ../../../RPUusb/UPDImode/UARTmode.py
```

## How To Use

Kicking the tires

Hopefully, the firmware upload worked; it should have switched the serial port connection back to UART mode so I can use picocom to interact with the AVR's USART0.

```bash
picocom -b 38400 /dev/ttyUSB0
[picocom -b 38400 /dev/ttyAMA0]
...
a
# exit is C-a, C-x
# if it aborts you can reset with pymcuprog
make reset
```

Using picocom send an 'a' to toggle the blinking (and ping i2c after that is working) on and off; after a few toggles, it will abort.

To test the i2c (TWI) I am using an RPUusb board that plugs into where the R-Pi would. It has a m328pb with i2c-debug software and is acceses from its second serial port /dev/ttyUSB1.

```bash
picocom -b 38400 /dev/ttyUSB1
...
/0/id?
{"id":{"name":"I2Cdebug^2","desc":"RPUusb (14145^5) Board /w ATmega328pb","avr-gcc":"5.4.0"}}
/0/iscan?
{"scan":[]}
/0/imon? 41
{"monitor_0x29":[{"data":"0x0"}]}
{"monitor_0x29":[{"data":"0x0"}]}
...
```

Ping TWI with each LED toggle; imon does not show updates if it is swamped with pings. The twi0_mc lib was first checked and then twi0_bsd was debuged and then ran (it is not much of a test but is a start).

The AVR128DB28 starts running at 24MHz/6 (4MHz) from the factory. To run at another frequency change the F_CPU define in the Makefile. The timers_bsd.c will select the correct option based on the F_CPU value passed to the compiler during the build.
