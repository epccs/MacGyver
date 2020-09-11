# MacGyver

From <https://github.com/epccs/MacGyver>


## Overview

The board name was PiUpdi, but that is confusing (with pyupdi), which was causing self-inflicted pain. I have mcgyvered (improvised use of IOFF buffer) the UPDI programming interface so that an Raspberry Pi pin can select it; in effect, it results in a UART mode or a UPDI mode from the R-Pi hardware serial port.

Board is used to connect a Raspberry Pi Zero (or W) hardware serial port (from 40 pin header) to ether the UPDI or UART0 port of an AVR128DA28. 

The Raspberry Pi hardware UART does not have latency like a USB-serial bridge, so its programming speed may be the best possible. Programing sends a lot of small sets of data back and forth; USB latency is almost certainly the cause of most complaints about UPDI speed.


## Status

![Status](./Hardware/status_icon.png "Status")


## [Hardware](./Hardware)

Hardware files include schematic and board images, bill of materials, and my notes on testing, evaluation, and schooling.


## Example

TBD


The Pi Zero needs a way to manualy [halt] from a push button, and start (when the AVR does not start it).

[halt]: ./Shutdown


## AVR toolchain

The shared files for this board are in the /lib folder. Each example has files and a Makefile in its folder. Hear are some toolchain links for referance (The AVR128DA should work with the packaged toolchain on R-Pi).

* toolchain: https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers
* source: https://www.microchip.com/mplab/avr-support/avr-and-sam-downloads-archive
* arduino7: wget http://downloads.arduino.cc/tools/avr-gcc-7.3.0-atmel3.6.1-arduino7-x86_64-pc-linux-gnu.tar.bz2
* Georg-Johann_Lay_10.0.0.pre: https://www.avrfreaks.net/forum/avr-gcc-64-bit-double

note: 10.0.0 has float/double/long_double (32/32/64).

```
# if side load tools are used skip packages: gcc-avr binutils-avr gdb-avr avr-libc 
sudo apt-get install git make avrdude gcc-avr binutils-avr gdb-avr avr-libc python3-pip
pip3 install pyserial intelhex pylint
pip3 install https://github.com/mraardvark/pyupdi/archive/master.zip
# [optional side loaded toolchain(s)]
# to side load avr8-gnu-toolchain-3.6.2.1759-linux.any.x86_64.tar.gz
wget https://www.microchip.com/mymicrochip/filehandler.aspx?ddocname=en607660
cp 'filehandler.aspx?ddocname=en607660' Samba/avr8-gnu-toolchain-3.6.2.1759-linux.any.x86_64.tar.gz
cd Samba
# <cynical>That was not obfuscated at all.</cynical>
# I got to this point from a remote Windows machine so sorry if the wget stuff does not work.
mkdir avr8-3.6.2
tar -xzvf avr8-gnu-toolchain-3.6.2.1759-linux.any.x86_64.tar.gz -C avr8-3.6.2
git clone https://github.com/epccs/MacGyver
# arduino has a toolchain form (bzip2 compression)
wget http://downloads.arduino.cc/tools/avr-gcc-7.3.0-atmel3.6.1-arduino7-x86_64-pc-linux-gnu.tar.bz2
# which is from https://github.com/arduino/toolchain-avr/tree/staging
tar -xjvf avr-gcc-7.3.0-atmel3.6.1-arduino7-x86_64-pc-linux-gnu.tar.bz2 -C avr-gcc-7.3.0-atmel3.6.1-arduino7
```

Some device-specific files from the [atpack] are also added to this repo.

[atpack]: http://packs.download.atmel.com/

With the m4809, I had to sideload the toolchain, since the packaged one lacked the xmega3 core. However, AVR128DA28 has an xmega4 core, and that is in older toolchains. I prefer using a package toolchain.


## BCM24 Cntl Uart/UPDI Mode

The R-Pi needs to control the BCM24 pin so that it can select the IOFF buffer needed for UPDI mode or UART mode. The scripts to do this are in [RPUusb/BCM24cntl]. The makefiles expects RPUusb to be loaded to the side of the MacGyver repo so clone both (for now).

[RPUusb/BCM24cntl]: https://github.com/epccs/RPUusb

```
git clone https://github.com/epccs/RPUusb
git clone https://github.com/epccs/MacGyver
```

Note that RPUusb has a MCU that will control the BCM24 pin and its UART is briged to the USB secondary UART. The USB primary UART is used for serial or UPDI depending on the selected setting from the secondary. 


## Application Notes

Microchip has been doing some [guides]. The m4809 has an xmega3 core, while the 128DA has an xmega4 core, so some differences should be expected.

[guides]: https://www.avrfreaks.net/forum/getting-started-attiny-1-0-series-application-notes

* [TB3229 - Getting Started with GPIO](http://ww1.microchip.com/downloads/en/Appnotes/90003229A.pdf)
* [TB3209 - Getting Started with ADC](http://ww1.microchip.com/downloads/en/AppNotes/TB3209-Getting-Started-with-ADC-90003209A.pdf)
* [TB3211 - Getting Started with AC](http://ww1.microchip.com/downloads/en/AppNotes/TB3211-Getting-Started-with-AC-90003211A.pdf)
* [TB3214 - Getting Started with TCB](http://ww1.microchip.com/downloads/en/AppNotes/TB3214-Getting-Started-with-TCB-90003214A.pdf)
* [TB3215 - Getting Started with SPI](http://ww1.microchip.com/downloads/en/AppNotes/TB3215-Getting-Started-with-SPI-90003215A.pdf)
* [TB3216 - Getting Started with USART](http://ww1.microchip.com/downloads/en/AppNotes/TB3216-Getting-Started-with-USART-90003216A.pdf)
* [TB3217 - Getting Started with TCA](http://ww1.microchip.com/downloads/en/AppNotes/TB3217-Getting-Started-with-TCA-90003217A.pdf)
* [TB3218 - Getting started with CCL](http://ww1.microchip.com/downloads/en/AppNotes/TB3218-Getting-Started-with-CCL-90003218A.pdf)
* [AN2451 - Getting Started with Core Independent Peripherals on AVR Microcontrollers](http://ww1.microchip.com/downloads/en/AppNotes/Getting-Started-with-Peripherals-on-AVR-MCU-00002451C.pdf)
* [AVR1000: Getting Started with Writing C-Code for XMEGA](http://ww1.microchip.com/downloads/en/AppNotes/doc8075.pdf)

Microchip also has examples on Github (and they are for the AVR128da)

https://github.com/search?q=org%3AMicrochipTech+avr128da&unscoped_q=avr128da

The Arduino m4809 I2C example can probably made to work on the AVR128DA's

https://github.com/arduino/ArduinoCore-megaavr/blob/master/libraries/Wire/src/utility/twi.c

If the UPDI idea has problems Optiboot could be a workaround

https://github.com/Optiboot/optiboot/blob/master/Wiki/CompilingOptiboot_x.md