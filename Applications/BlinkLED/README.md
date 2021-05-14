# BlinkLED

## To Do

twi0 ping is off, the ping would send to managers twi1 but that is yet to be done.

## Overview

Kick the tires.

## Firmware Upload

Use a UPDI upload tool. Run 'make' to compile and 'make updi' to upload.

```bash
sudo apt-get install make git gcc-avr binutils-avr gdb-avr avr-libc python3-pip
pip3 install pymcuprog
git clone https://github.com/epccs/MacGyver/
cd /MacGyver/Applications/BlinkLED
make all
...
make updi
testing for prerequesetits a false will stop make
which python3 2>/dev/null || false
/usr/bin/python3
which pymcuprog 2>/dev/null || false
/home/rsutherland/.local/bin/pymcuprog
ls ../../../MacGyver/Manager/AppUpload/AppUPDImode.py 2>/dev/null || false
../../../MacGyver/Manager/AppUpload/AppUPDImode.py
ls ../../../MacGyver/Manager/AppUpload/AppUARTmode.py 2>/dev/null || false
../../../MacGyver/Manager/AppUpload/AppUARTmode.py
python3 ../../../MacGyver/Manager/AppUpload/AppUPDImode.py
bootmsg: i2c-debug at addr 48: commands include
bootmsg: /0/id?
bootmsg: 
cmd echo: /0/iaddr 42
response: {"master_address":"0x2A"}
#: Manager/AppUpload firmware looks for a 7 on i2c to enable Application UPID mode
cmd echo: /0/ibuff 7
response: {"txBuffer[1]":[{"data":"0x7"}]}
cmd echo: /0/iread? 1
response: {"txBuffer":"wrt_success","rxBuffer":"rd_success","rxBuffer":[{"data":"0x7"}]}
pymcuprog erase -t uart -u /dev/ttyUSB0 -d avr128da28
Chip/Bulk erase,
Memory type eeprom is conditionally erased (depending upon EESAVE fuse setting)
Memory type flash is always erased
Memory type internal_sram is always erased
Memory type lockbits is always erased
...
Erased.
Done.
pymcuprog write -t uart -u /dev/ttyUSB0 -d avr128da28 --verify -f BlinkLED.hex
Writing from hex file...
Writing flash...
Verifying flash...
OK
Done.
python3 ../../../MacGyver/Manager/AppUpload/AppUARTmode.py
bootmsg: i2c-debug at addr 48: commands include
bootmsg: /0/id?
bootmsg: 
cmd echo: /0/iaddr 42
response: {"master_address":"0x2A"}
#: Manager/AppUpload firmware looks for anything except 7 on i2c to enable Application UART mode
cmd echo: /0/ibuff 0
response: {"txBuffer[1]":[{"data":"0x0"}]}
cmd echo: /0/iread? 1
response: {"txBuffer":"wrt_success","rxBuffer":"rd_success","rxBuffer":[{"data":"0x0"}]}
```

## How To Use

Kicking the tires

Hopefully, the firmware upload worked; it should have switched the serial port connection back to UART mode so I can use picocom to interact with the AVR's USART0.

```bash
[picocom -b 38400 /dev/ttyUSB0]
picocom -b 38400 /dev/ttyAMA0
...
a
# exit is C-a, C-x
# if it aborts you can reset with pyupdi
make reset
```

An 'a' will toggle the blinking and i2c ping (off), and another will toggle it on; an '$' will abort.

To test the i2c (TWI) I will add a monitor to output on the manager debug interface.

```bash
picocom -b 38400 /dev/ttyUSB0
...
TBD
...
```

The AVR128DA28 starts running at 24MHz/6 (4MHz) from the factory. To run at another frequency, we can change the clock select bit field (e.g., CLKCTRL_FREQSEL_16M_gc). The timers_bsd.c will select the correct option based on the F_CPU value that the Makefile passes to the compiler during the build.
