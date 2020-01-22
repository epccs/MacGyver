# PiUpdi

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

The shared files for this board are in the /lib folder. Each example has files and a Makefile in its folder. Since the packaged toolchain does not support the m4809 device, I will evaluate it with the Microchip [toolchain] sideloaded into a folder in my home directory (~/Samba/avr8-3.6.2). Note the [source] (atm AVR GCC 3.6.2) for the new toolchain has been moved from where Atmel use to keep them.

[toolchain]: https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers
[source]: https://www.microchip.com/mplab/avr-support/avr-and-sam-downloads-archive

```
# the side load will do these packages gcc-avr binutils-avr gdb-avr avr-libc 
sudo apt-get install git make avrdude
# wget avr8-gnu-toolchain-3.6.2.1759-linux.any.x86_64.tar.gz
wget https://www.microchip.com/mymicrochip/filehandler.aspx?ddocname=en607660
cp 'filehandler.aspx?ddocname=en607660' Samba/avr8-gnu-toolchain-3.6.2.1759-linux.any.x86_64.tar.gz
cd Samba
# I got to this point from a remote Windows machine so sorry if the wget stuff does not work.
mkdir avr8-3.6.2
tar -xzvf avr8-gnu-toolchain-3.6.2.1759-linux.any.x86_64.tar.gz -C avr8-3.6.2
git clone https://github.com/epccs/PiUpdi
```

I also included some device-specific files from the [atpack] in my repository.

[atpack]: http://packs.download.atmel.com/

I prefer using a package toolchain, this is almost enough pain to prefer AS7 (but that is a dead end tool).


## Application Notes for ATmega4809

Microchip has been doing some [guides].

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

Microchip also has some examples on Github

https://github.com/MicrochipTech/TB3216_Getting_Started_with_USART

I2C example

https://github.com/arduino/ArduinoCore-megaavr/blob/master/libraries/Wire/src/utility/twi.c

If the UPDI idea has problems Optiboot could be a workaround not that it support the m4809

https://github.com/Optiboot/optiboot/blob/master/Wiki/CompilingOptiboot_x.md