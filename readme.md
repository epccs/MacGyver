# MacGyver

From <https://github.com/epccs/MacGyver>


## Overview

This board allows a Raspberry Pi serial hardware port (or adaptor board) to interface with a multi-drop. A local AVR128DA28 is on the multi-drop. Programing is done through the serial interface when a target (e.g., the local AVR128DA28) is set by the manager for UPDI (the programming method for new AVRs) or UART mode.

Multi-drop serial allows the Raspberry Pi to reach other boards connected with a CAT5 cable. The managers have an out of band channel for the connection state (bus state) so that a host (R-Pi) can establish a point to point connection to the remote UPDI port. Regular serial communication can be a point to point or point to multi-point.

I have improvised some IOFF buffers so the UPDI programming interface can be selected; it allows selecting a UART mode or a UPDI mode. A Raspberry Pi hardware UART is ideal since it does not have latency like a USB-serial bridge; the programming speed is the best possible. Programing sends many small sets of data back and forth; USB latency reduces the apparent UPDI speed. UPDI programming allows the use of any fuse, code protection, and functional safety feature. An erase command will clear the part so another program and fuse combination can be loaded. Thus bricking is not possible, but you can wear out the flash memory (rated for 10k writes).


## Status

![Status](./Hardware/status_icon.png "Status")


## [Hardware](./Hardware)

Hardware files include schematic and board images, bill of materials, and my notes on testing, evaluation, and schooling.


## Example

TBD

The firmware that runs on this board is written in a general-purpose language C or assembly (C++ may also work if that is your thing, I have been avoiding it). The program needs careful reviewing and then compiled into the binary instructions that comprise the firmware. I recommend dividing the software into parts that are as minimal as possible for testing. Testing and ensuring correctness is the user's responsibility. PLCs are for anticipated Industrial Applications, but if what you are doing is not in their tool bag, or if you plan to make more than a handful, then it may be worth considering an MCU with functional safety features.

https://library.automationdirect.com/microcontrollers-versus-plcs-theres-a-clear-winner-for-your-industrial-applications/

The UART0 on the application controller is used for the multi-drop serial bus that is interfaced with the serial hardware of the Single Board Computer (SBC). The 40 pin header of a Raspberry Pi uses pins 8 and 10 for RX and TX; other SBC's should also work (I do not test them). The RJ45 connectors are for the multi-drop serial bus daisy chain connection with terminations at the ends. The pairs are done the same as ethernet, so I can swap around cables if needed, but don't accidentally connect PoE devices since that would probably let out some smoke. 

The Raspberry Pi will start at power-up, a switch on the board is used to [halt] it.

[halt]: ./Shutdown


## AVR toolchain

This board uses the AVR toolchain. I use the one from Microchip that has been packaged for Debian (3.6.1, it is also on Ubuntu, Raspbian, and Windows with WSL). With the toolchain installed, the AVR application can compile localy. 

The frequently used files for this board are in the /lib folder (in Applicaions and Manager). Each example application has its files and a Makefile in a separate folder (in Applicaions and Manager). The toolchain is also available as packages. 

* toolchain: https://www.microchip.com/mplab/avr-support/avr-and-arm-toolchains-c-compilers
* source: https://www.microchip.com/mplab/avr-support/avr-and-sam-downloads-archive
* arduino7: wget http://downloads.arduino.cc/tools/avr-gcc-7.3.0-atmel3.6.1-arduino7-x86_64-pc-linux-gnu.tar.bz2
* Georg-Johann_Lay_10.0.0.pre: https://www.avrfreaks.net/forum/avr-gcc-64-bit-double

note: 10.0.0 has float/double/long_double (32/32/64).

```
# the modem manager causes problme with my old programer but not with the R-Pi uart
[sudo apt-get purge modemmanager]
# if side load tools are used skip packages: gcc-avr binutils-avr gdb-avr avr-libc 
sudo apt-get install git make avrdude gcc-avr binutils-avr gdb-avr avr-libc python3-pip
pip3 install pyserial intelhex pylint
pip3 install https://github.com/mraardvark/pyupdi/archive/master.zip
# new (atm) uploader from Microchip https://pypi.org/project/pymcuprog/
pip3 install pymcuprog
# [<optional> side loaded toolchain(s)
#   to side load avr8-gnu-toolchain-3.6.2.1759-linux.any.x86_64.tar.gz
   wget https://www.microchip.com/mymicrochip/filehandler.aspx?ddocname=en607660
   cp 'filehandler.aspx?ddocname=en607660' Samba/avr8-gnu-toolchain-3.6.2.1759-linux.any.x86_64.tar.gz
   cd Samba
#    <cynical>That was not obfuscated at all.</cynical>
#    I got to this point from a remote Windows machine so sorry if the wget stuff does not work.
    mkdir avr8-3.6.2
    tar -xzvf avr8-gnu-toolchain-3.6.2.1759-linux.any.x86_64.tar.gz -C avr8-3.6.2
    git clone https://github.com/epccs/MacGyver
#    arduino has a toolchain form (bzip2 compression)
    wget http://downloads.arduino.cc/tools/avr-gcc-7.3.0-atmel3.6.1-arduino7-x86_64-pc-linux-gnu.tar.bz2
#    which is from https://github.com/arduino/toolchain-avr/tree/staging
    tar -xjvf avr-gcc-7.3.0-atmel3.6.1-arduino7-x86_64-pc-linux-gnu.tar.bz2 -C avr-gcc-7.3.0-atmel3.6.1-arduino7
# </optional> side loaded toolchain(s)]
```

Some device-specific files are from the [atpack] also added to this repo.

[atpack]: http://packs.download.atmel.com/

The AVR128DA28 has an avrxmega4 arch, and that is in the 3.6.1 toolchain. It is easy to use a packaged toolchain.

The compiler does not implement exception handling for dynamic memory allocation corruptions (e.g., when the stack and heap collide), so it is ill-advised to use dynamic allocation (heap) on this device. The C++ STL is full of dynamic allocation; much of the C++ found on the internet also uses the heap; I have noticed open-source projects done with C (and lack dynamic allocation) work better on these type of devices, which do not seperate the stack memory from the heap memory.


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
"defines": ["__AVR_DEV_LIB_NAME__=avr128da28","__AVR_ARCH__=104","__AVR_XMEGA__=1","F_CPU=16000000UL"],
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

The compiler has a long list of builtin (hidden) defines, and some need to be added to VScode c_cpp_properties.json

```
make builtin
```


## Field Updates

There is no bootloader. The basic idea is that the R-Pi can upload firmware to an AVR128DB32 UPDI directly to act as a manager that can then link UPDI through a multi-drop full-duplex connection and load firmware to the AVR128DA28. The DA28 should be easy to replace, so the DIP socket is perfect.

Updating flash from memory devices like SD cards has been a traditional way to do field updates. The update may be sent to the user as an SD card (or something similar) and then plugged into the product in the field to upgrade it. The idea is that providing an in-circuit programmer and a script to control that would be difficult; the customer would have to (climb up the tree and) access the products with their laptop and the programming cable to update them. An SD card eliminates the fiddly cable, but updates are still a nightmare. One of these boards could be remote (in the tree) without an R-Pi, and a CAT5 line could run (down) to an encloser (near the base) for access. When updates or data access are needed, another of these boards with an R-Pi (or RPUusb) could do what is required.

A more exciting method is to connect a group of units to one that has an R-Pi permanently attached. If the R-Pi is near a WiFi bridge that goes to a Starlink connection, the options become very extensible. A setup might look like a hub with CAT5 lines radiating out to the maximum distance that the RS485 transceivers allow. 

The UPDI interface is not a bootloader interface; it is the programming interface provided by the hardware manufacturer and is likely to be implemented correctly, more so than the bootloader I could implement. If fuses get set that lock the device, it can be erased for a do-over. Scrips can be linked-to desktop icons for ease of use, and the fiddley parts of in-circuit programming have to do with the cabling, not the upload software (which gets scripted).

What about the ever-changing near future of a ranch water source? They deliver water to sites downslope through durable, low-cost HDPE plumbing, but sometimes it breaks or is blocked. Assuming the well has the primary power source, it would be the best location for a Starlink connection and some boards like this one (actually the Irrigate7 board after it gets overhauled) connected to control and monitor the reasonably nearby assets (main tank level, pump, battery charge, and valves that send water to remote locations). At the same time, a LoRa gateway is used to measure faraway assets (watering trough levels in remote areas). Controlling the valves near the source and measuring the volume dispensed can prevent emptying the tank when something breaks and allows an automatic siphon to clean a water trough before partly refilling. The important thing is ideas just keep adding, and various parts of the control software need to be changed to accommodate, so updates are a vital component of most automation.