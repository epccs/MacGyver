# AppUpload

## ToDo

The UPDImode.py script needs some attention, the output is out of sync.

## Overview

This firmware will allow the manager (AVR128DB32) to enable the transceivers to pass the host serial to the application (AVR128DA), also connect the application side of the multi-drop serial to its UPDI pin. The Out Of Band channel is disabled.

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

Kicking more of the tires. The ttyUSB0 is connected to the multi-drop, while in UART mode the application controler will do nil when unprogramed, but may provide a response if programed to do so.

```bash
picocom -b 38400 /dev/ttyUSB0
[picocom -b 38400 /dev/ttyAMA0]
...
a
# exit is C-a, C-x
# if it aborts you can reset with "make reset"
```

The ttyUSB1 is connected to a m328pb on the RPUusb which can control some the pins in place of an R-Pi, includeing the manager's twi0 through i2c-debug software on m328pb. Send a seven (command) to cause the manager to set the application in UPDI programing mode, and send anything else to switch back to UART mode.

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
/0/ibuff 7
{"txBuffer[1]":[{"data":"0x7"}]}
/0/iread? 1
{"txBuffer":"wrt_success","rxBuffer":"rd_success","rxBuffer":[{"data":"0x7"}]}
# notice the LED blink rate is 4x faster on manager to show it is has put applicaiton into UPDI mode
/0/ibuff 0
{"txBuffer[1]":[{"data":"0x0"}]}
/0/iread? 1
{"txBuffer":"wrt_success","rxBuffer":"rd_success","rxBuffer":[{"data":"0x0"}]}
# notice the LED blink rate is back to normal on manager to show it is has put applicaiton into UART mode
# the appliction TX0 pin may be floating and seen as a LOW through the TX pair and into the host RX input.
```
