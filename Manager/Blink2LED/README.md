# BlinkLED

## ToDo

Use the twi0_mc as slave, since I can't get twi0_bsd master to work, I need to sort out the twi0_mc slave.

## Overview

UART1 is on PORTC, which I plan to use for debugging. I copied timers_bsd.h from the application (AVR128DA) in which I had routed the TCA0 PWMs to PORTC. That caused a lot of confusion since enabling the TX1 pin as an output caused a PWM output.

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
pymcuprog erase -t uart -u /dev/ttyUSB0 -d avr128db32
Chip/Bulk erase,
Memory type eeprom is conditionally erased (depending upon EESAVE fuse setting)
Memory type flash is always erased
Memory type internal_sram is always erased
Memory type lockbits is always erased
...
Erased.
Done.
pymcuprog write -t uart -u /dev/ttyUSB0 -d avr128db32 --verify -f BlinkLED.hex
Writing from hex file...
Writing flash...
Verifying flash...
OK
Done.
python3 ../../../RPUusb/UPDImode/UARTmode.py
```

[https://github.com/microchip-pic-avr-tools/pymcuprog]

## How To Use

Kicking the tires.

After the firmware upload the mulit-drop connection should have switched back to UART mode, which does nothing at this time. The manager debug port does not connect to this, it will have test points that may be used for development/test.

```bash
picocom -b 38400 /dev/ttyUSB0
[picocom -b 38400 /dev/ttyAMA0]
...
# exit is C-a, C-x
# if it aborts you can reset with pymcuprog
make reset
```

To test the i2c (TWI) between the manager and SBC, I am using an RPUusb board that plugs into where the R-Pi would. It has an m328pb with i2c-debug software and is accessed from its second serial port /dev/ttyUSB1.

```bash
picocom -b 38400 /dev/ttyUSB1
...
Terminal ready
i2c-debug at addr 48: commands include
/0/id? 
/0/iscan?
{"scan":[{"addr":"0x2A"}]}
/0/iaddr 42
{"master_address":"0x2A"}
/0/ibuff 2
{"txBuffer[1]":[{"data":"0x2"}]}
/0/iread? 1
{"txBuffer":"wrt_success","rxBuffer":"rd_success","rxBuffer":[{"data":"0x2"}]}
/0/ibuff 127,15
{"txBuffer[2]":[{"data":"0x7F"},{"data":"0xF"}]}
/0/iread? 2
{"txBuffer":"wrt_success","rxBuffer":"rd_success","rxBuffer":[{"data":"0x7F"},{"data":"0xF"}]}
# send "a" on manager debug to stop blink
# send "$" on manarer debug to abort
/0/iscan?
{"scan":[]}
# scan finished because abort turned off I2C.
```

At the same time a USBuart board is connected to the manager debug port /dev/ttyUSB2. The scan has to write a byte to each address. Send an 'a' to toggle the blinking on and off; send an '$' to abort, other keys echo.

```bash
picocom -b 38400 /dev/ttyUSB2
...
Terminal ready
{"monitor_0x2A":[{"status":"0"},{"len":"0"}]}
{"monitor_0x2A":[{"status":"3"},{"len":"1"},{"dat":"0x2"}]}
{"monitor_0x2A":[{"status":"19"},{"len":"2"},{"dat":"0x7F"},{"dat":"0xF"}]}
a
{"abort":"'$' found"}
```

The status bits are: Data ISR Flag[7], AP ISR Flag, Clock Hold, Received Ack, Collision, Bus Error, R/W Direction, AP[0]

TWI0.SSTATUS is saved in transmit_callback and reported as status. It shows a range of flags set at the time of the callback.

The AVR128DB28 starts running at 24MHz/6 (4MHz) from the factory. To run at another frequency change, the F_CPU define in the Makefile. The timers_bsd.c will select the correct option based on the F_CPU value passed to the compiler during the build. That is unusual for AVR but works on these new devices; it is also worth noting that if the clock fails (perhaps due to supply voltage), it will switch back to 4MHz, which operates at all valid voltages.
