# AppUpload

## ToDo

Address 42 on manager TWI0 (between manager and R-Pi) takes an i2c write byte, so an SMBus command may not work \\_(:-/)_/.

## Overview

This firmware will allow the manager (AVR128DB32) to enable the transceivers to pass the host serial to the application (AVR128DA), or connect the application side of the multi-drop serial to its UPDI pin. The Out Of Band channel is disabled. Only a write+read on i2c from the host can change the UPDI mode for Application programing, a write alone is ignored, the read is an echo of the write.

## Firmware Upload

Uses a UPDI upload tool. Run 'make' to compile and 'make updi' to upload.

```bash
sudo apt-get install make git gcc-avr binutils-avr gdb-avr avr-libc python3-pip
pip3 install pymcuprog
git clone https://github.com/epccs/MacGyver/
cd /MacGyver/Manager/AppUpload
make all
...
make updi
...
Erased.
Done.
pymcuprog write -t uart -u /dev/ttyUSB0 -d avr128db32 --verify -f AppUpload.hex
Writing from hex file...
Writing flash...
Verifying flash...
OK
Done.
...
```

[https://github.com/microchip-pic-avr-tools/pymcuprog]

## How To Use

Kicking more of the tires. The ttyUSB0 is connected to the multi-drop, while in UART mode the application controler will do nil if unprogramed, but may provide a response if programed to do so (see [Applications] )

[Applications]:../../Applications

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
