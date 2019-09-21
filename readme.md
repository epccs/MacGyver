# RPUpi

From <https://github.com/epccs/PiUpdi>

## Overview

Board used to connect a Raspberry Pi Zero (or W) 40 pin header to serial or UPDI port of an ATmega4809. 


## Status

![Status](./Hardware/status_icon.png "Status")


## [Hardware](./Hardware)

Hardware files include schematic and board images, bill of materials, and my notes on testing, evaluation, and schooling.


## Example

TBD


The Pi Zero needs a way to manualy [halt] from a push button, and start (when the AVR fails to start it).

[halt]: ./Shutdown


## AVR toolchain

The core files for this boards bus manager are in the /lib folder. Each example has its files and a Makefile in its own folder. The toolchain packages that I use are available on Ubuntu and Raspbian. 

```
sudo apt-get install git make gcc-avr binutils-avr gdb-avr avr-libc avrdude
git clone https://github.com/epccs/PiUpdi
```

* [gcc-avr](https://packages.ubuntu.com/search?keywords=gcc-avr)
* [binutils-avr](https://packages.ubuntu.com/search?keywords=binutils-avr)
* [gdb-avr](https://packages.ubuntu.com/search?keywords=gdb-avr)
* [avr-libc](https://packages.ubuntu.com/search?keywords=avr-libc)
* [avrdude](https://packages.ubuntu.com/search?keywords=avrdude)