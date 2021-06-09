# BlinkLED

## Overview

I have had little luck with the TWI peripheral libraries I have ported to this chip, so it is time to look at other options.

This is from <https://github.com/cv007/Avr01Dx_Twi>

## Firmware Upload

Uses the pymcuprog tool. Run 'make all' to compile and 'make updi' to upload.

```bash
sudo apt-get install make git gcc-avr binutils-avr gdb-avr avr-libc python3-pip
pip3 install pymcuprog
git clone https://github.com/epccs/MacGyver/
cd /MacGyver/Applications/Avr01Dx_Twi-main
make all
...
# works only on my MacGyver board with a RPUusb (where the R-Pi goes)
make updi
...
```

## How To Use

Get out your Oscilloscope, it helps to have one with i2c serial decode (note on an Agilent 2k series 51~A means the address did not get Ack)
