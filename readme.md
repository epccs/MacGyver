# MacGyver

From <https://github.com/epccs/MacGyver>


## Overview

This board allows a Raspberry Pi serial hardware port (or adaptor board) to interface with a multi-drop. A local AVR128DA28 is on the multi-drop. Programing is done through the serial interface when a target (e.g., the local AVR128DA28) is set by the manager for UPDI (the preferred programing method for new AVRs) or UART mode.

I have improvised (mcgyvered) the use of IOFF buffers so the UPDI programming interface can be selected; it results in a UART mode or a UPDI mode from the R-Pi hardware serial port.

Board is used to connect a Raspberry Pi hardware serial port (e.g., 40 pin header) to ether the UPDI or UART0 port of an AVR128DB32 manager, or (with help from the manager) to a local AVR128DA28 on the multi-drop serial. 

The Raspberry Pi hardware UART does not have latency like a USB-serial bridge, so its programming speed is the best possible. Programing sends a lot of small sets of data back and forth; USB latency is almost certainly the cause of most complaints about UPDI speed.


## Status

![Status](./Hardware/status_icon.png "Status")


## [Hardware](./Hardware)

Hardware files include schematic and board images, bill of materials, and my notes on testing, evaluation, and schooling.


## Example

TBD


The a switch on the board is used to [halt] the Raspberry Pi it will start at power up.

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
sudo apt-get purge modemmanager
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


## BCM24 Cntl UPDI mode and BCM23 Cntl UART mode

The R-Pi (or RPUusb) controls the BCM23 and BCM24 pins so that the UART and UPDI modes can be selected. At this time, the scripts to control the pins are in [RPUusb/BCM24cntl]. The makefiles expect the RPUusb repository to be loaded to the side of the MacGyver repo, so clone both.

[RPUusb/UPDImode]: https://github.com/epccs/RPUusb/tree/master/UPDImode

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


## VSCode

There are two folders with independent ".vscode" setup, the Manager and Applications folders. Each uses different microcontrollers and needs unique defines. In the Applications/.vscode/c_cpp_properties.json, I put these defines.

```
"defines": ["__AVR_DEV_LIB_NAME__=avr128da28","F_CPU=16000000UL"],
```

The __AVR_DEV_LIB_NAME__ is from the -mmcu compiler option. Intellisense should track it through #include <avr/io.h> which would then include "io" + "avr128da28" + ".h". Intellisense does not look outside the includes origin (cross-origin), so it reports an error about not finding ioavr128da28.h, therefor the MCU header has to be put where it can find it. I have added a rule to do this (it needs admin rights).

```
sudo make hdr4code
```

After that I removed the forcedInclude I was using

```
"forcedInclude": [ "${workspaceFolder}/lib/AVR-Dx_DFP/include/avr/ioavr128da28.h" ],
```

Code is now seeing the same thing that the compiler does, and not recusing though files multiple times (and leaking as it does) to build its database.

https://blog.robenkleene.com/2020/09/21/the-era-of-visual-studio-code/


## Field Updates

Updating flash from memory devices like SD cards has been a traditional way to do field updates. The update may be sent to the user as an SD card (or something similar) and then plugged into the product in the field to upgrade it. The idea is that providing an in-circuit programmer and a script to control that would be difficult; the customer would have to (climb up the tree and) access the products with their laptop and the programming cable to update them. An SD card eliminates the fiddly cable, but updates are still a nightmare. One of these boards could be remote (in the tree) without an R-Pi, and a CAT5 line could run (down) to an encloser (at the base) for access. When updates or data access are needed, another of these boards with an R-Pi (or RPUusb) could do what is required.

A more exciting method is to connect a group of units to one that has an R-Pi permanently attached. If the R-Pi is near a WiFi bridge that goes to a Starlink connection, the options become very extensible. A setup might look like a hub with CAT5 lines radiating out to the maximum distance that the RS485 transceivers allow. 

The UPDI interface is not a bootloader interface; it is the programming interface provided by the hardware manufacturer and is likely to be implemented correctly, more so than the bootloader I could implement. If fuses get set that lock the device, it can be erased for a do-over. Scrips can be linked-to desktop icons for ease of use, and the fiddley parts of in-circuit programming have to do with the cabling, not the upload software (which gets scripted).

What about the ever-changing near future of a ranch water source? They deliver water to sites downslope through durable, low-cost HDPE plumbing, but sometimes it breaks or is blocked. Assuming the well has the primary power source, it would be the best location for a Starlink connection and some boards like this one (actually the Irrigate7 board after it gets overhauled) connected to control and monitor the reasonably nearby assets (main tank level, pump, battery charge, and valves that send water to remote locations). At the same time, a LoRa gateway is used to measure faraway assets (watering trough levels in remote areas). Controlling the valves near the source and measuring the volume dispensed can prevent emptying the tank when something breaks and allows an automatic siphon to clean a water trough before partly refilling. The important thing is ideas just keep adding, and various parts of the control software need to be changed to accommodate, so updates are a vital component of most automation.