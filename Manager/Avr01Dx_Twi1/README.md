# Twi01Dx

## Overview

I have had little luck with the TWI peripheral libraries I have ported to this chip, so it is time to look at other options.

This is from <https://github.com/cv007/Avr01Dx_Twi>

## Firmware Upload

 Run 'make all' to compile and 'make updi' to upload.

```bash
sudo apt-get install make git gcc-avr binutils-avr gdb-avr avr-libc python3-pip
pip3 install pymcuprog
git clone https://github.com/epccs/MacGyver/
cd /MacGyver/Manager/Avr01Dx_Twi1
make all
avr-gcc -Os -g -std=gnu99 -Wall -fshort-enums -ffunction-sections -fdata-sections  -DF_CPU=16000000UL -I.  -mmcu=avr128db32 -B ../lib/AVR-Dx_DFP/gcc/dev/avr128db32/ -I ../lib/AVR-Dx_DFP/include/ -c -o main.o main.c
avr-gcc -Os -g -std=gnu99 -Wall -fshort-enums -ffunction-sections -fdata-sections  -DF_CPU=16000000UL -I.  -mmcu=avr128db32 -B ../lib/AVR-Dx_DFP/gcc/dev/avr128db32/ -I ../lib/AVR-Dx_DFP/include/ -c -o ds3231.o ds3231.c
avr-gcc -Os -g -std=gnu99 -Wall -fshort-enums -ffunction-sections -fdata-sections  -DF_CPU=16000000UL -I.  -mmcu=avr128db32 -B ../lib/AVR-Dx_DFP/gcc/dev/avr128db32/ -I ../lib/AVR-Dx_DFP/include/ -c -o twi1m.o twi1m.c
avr-gcc -Os -g -std=gnu99 -Wall -fshort-enums -ffunction-sections -fdata-sections  -DF_CPU=16000000UL -I.  -mmcu=avr128db32 -B ../lib/AVR-Dx_DFP/gcc/dev/avr128db32/ -I ../lib/AVR-Dx_DFP/include/ -c -o twi1s.o twi1s.c
avr-gcc -Os -g -std=gnu99 -Wall -fshort-enums -ffunction-sections -fdata-sections  -DF_CPU=16000000UL -I.  -mmcu=avr128db32 -B ../lib/AVR-Dx_DFP/gcc/dev/avr128db32/ -I ../lib/AVR-Dx_DFP/include/ -c -o ../lib/timers_bsd.o ../lib/timers_bsd.c
avr-gcc -Os -g -std=gnu99 -Wall -fshort-enums -ffunction-sections -fdata-sections  -DF_CPU=16000000UL -I.  -mmcu=avr128db32 -B ../lib/AVR-Dx_DFP/gcc/dev/avr128db32/ -I ../lib/AVR-Dx_DFP/include/ -c -o ../lib/uart1_bsd.o ../lib/uart1_bsd.c
avr-gcc -Wl,-Map,Twi01Dx.map  -Wl,--gc-sections  -mmcu=avr128db32 -B ../lib/AVR-Dx_DFP/gcc/dev/avr128db32/ -I ../lib/AVR-Dx_DFP/include/ main.o ds3231.o twi1m.o twi1s.o ../lib/timers_bsd.o ../lib/uart1_bsd.o -o Twi01Dx.elf
avr-size Twi01Dx.elf
   text    data     bss     dec     hex filename
   5550      14     142    5706    164a Twi01Dx.elf
rm -f Twi01Dx.o main.o ds3231.o twi1m.o twi1s.o ../lib/timers_bsd.o ../lib/uart1_bsd.o
avr-objcopy -j .text -j .data -O ihex Twi01Dx.elf Twi01Dx.hex
avr-objdump -h -S Twi01Dx.elf > Twi01Dx.lst
```

Upload uses the pymcuprog tool, the upload only works with my MacGyver board and a RPUusb board (where the R-Pi goes) at this time.

```bash
testing for prerequesetits a false will stop make
which python3 2>/dev/null || false
/usr/bin/python3
which pymcuprog 2>/dev/null || false
/home/rsutherland/.local/bin/pymcuprog
ls ../../../MacGyver/Manager/AppUpload/MgrUPDImode.py 2>/dev/null || false
../../../MacGyver/Manager/AppUpload/MgrUPDImode.py
ls ../../../MacGyver/Manager/AppUpload/MgrUARTmode.py 2>/dev/null || false
../../../MacGyver/Manager/AppUpload/MgrUARTmode.py
python3 ../../../MacGyver/Manager/AppUpload/MgrUPDImode.py
bootmsg: i2c-debug at addr 48: commands include
bootmsg: /0/id?
bootmsg: 
cmd echo: /0/updi
response: {"PiUPDI":"UPDI"}
pymcuprog erase -t uart -u /dev/ttyUSB0 -d avr128db32
Chip/Bulk erase,
Memory type eeprom is conditionally erased (depending upon EESAVE fuse setting)
Memory type flash is always erased
Memory type internal_sram is always erased
Memory type lockbits is always erased
...
Erased.
Done.
pymcuprog write -t uart -u /dev/ttyUSB0 -d avr128db32 --verify -f Twi01Dx.hex
Writing from hex file...
Writing flash...
Verifying flash...
OK
Done.
python3 ../../../MacGyver/Manager/AppUpload/MgrUARTmode.py
```

## How To Use

Get out your Oscilloscope, it helps to have one with i2c serial decode (note on an Agilent 2k series 51R~A means the address did not get Ack)

Not ack by ... lets call it the server to be PC.

![address_not_ack_by_slave](./address_not_ack_by_slave.jpg)

Both the client and server (PC) are running on the same chip, try that with a 328p.

![address_ack_by_slave](./address_ack_by_slave.jpg)

Note for futrue self: I ran with the Application running BlinkLED which sends a packet every other toggle, there were collisions.

```text
ticks 5, tmo 0, short 0, good 59328, bad 17
ticks 5, tmo 0, short 0, good 59331, bad 17
ticks 5, tmo 0, short 0, good 59333, bad 17
ticks 5, tmo 0, short 0, good 59336, bad 17
ticks 5, tmo 0, short 0, good 59339, bad 17
ticks 5, tmo 0, short 0, good 59342, bad 17
ticks 5, tmo 0, short 0, good 59345, bad 17
ticks 5, tmo 0, short 0, good 59348, bad 17
```
