# EEPROM

## Todo

EEPROM is WIP, but reading/writing a byte is working now. 
The referances are going to be held on the manager. The application eeprom is for the user.

## Overview

Eeprom is an interactive command line program that demonstrates the control of the EEPROM.


# Firmware Upload

TBD, but at this time the updi rule needs some python scripts in RPUusb

``` 
sudo apt-get install make git picocom gcc-avr binutils-avr gdb-avr avr-libc avrdude
git clone https://github.com/epccs/RPUusb/
git clone https://github.com/epccs/MacGyver/
cd /MacGyver/Applications/Eeprom
make all
make updi
...
Programming successful
...
INFO:phy Closing port '/dev/ttyUSB0'
python3 ../../../RPUusb/UPDImode/UARTmode.py
``` 

Now connect with picocom (or ilk).


``` 
#exit is C-a, C-x
picocom -b 38400 /dev/ttyUSB0
#picocom -b 38400 /dev/ttyAMA0
``` 

or log the terminal session

``` 
script -f -c "picocom -b 38400 /dev/ttyUSB0" stuff.log
``` 

# Commands

Commands are interactive over the serial interface at 38400 baud rate. The echo will start after the second character of a new line. 


## /\[rpu_address\]/\[command \[arg\]\]

rpu_address is taken from the manager (e.g. get_Rpu_address() which is included form ../lib/rpu_mgr.h). The value of rpu_address is used as a character in a string, which means don't use a null value (C strings are null terminated) as an adddress. The ASCII value for '1' (0x31 or 49) is easy and looks nice.

Commands and their arguments follow.


## /0/id? \[name|desc|avr-gcc\]

identify 

``` 
/0/id?
{"id":{"name":"Eeprom","desc":"MacGyver (19260^1) Board /w AVR128DA28","avr-gcc":"5.4.0"}}
```

##  /0/ee? address\[,type\]

Return the EEPROM value at address [0..511]. Type is UINT8, UINT16 or UINT32. Default type is UINT8. This checks if eeprom_is_ready() befor trying to read the EEPROM, if not ready it loops back through the program. 

``` 
/0/ee? 0
{"EE[0]":{"r":"255"}}
/0/ee? 0,UINT8
{"EE[0]":{"r":"255"}}
/0/ee? 0,UINT16
{"EE[30]":{"r":"65535"}}
/0/ee? 0,UINT32
{"EE[0]":{"r":"4294967295"}}
```

Note: The numbers are packed little endian by the gcc compiler (AVR itself has no endianness).


##  /0/ee address,value\[,type\]

Write the value to the address [0..511] as type. Type is Type is UINT8, UINT16 or UINT32. Default type is UINT8. This checks if eeprom_is_ready() befor trying to read the EEPROM, if not ready it loops back through the program. The JSON response is a read of the EEPROM. 

__Warning__ writing EEPROM can lead to device failure, it is only rated for 100k write cycles.

``` 
/0/ee 0,25
{"EE[0]":{"byte":"25","r":"25"}} 
/0/ee 1,128,UINT8
{"EE[1]":{"byte":"128","r":"128"}}
/0/ee? 1,UINT8
{"EE[1]":{"r":"128"}}
## so next is WIP
/0/ee 2,65535,UINT16
tbd
/0/ee 0,4294967295,UINT32
tbd
```
Note: 4294967295 is 0xFFFFFFFF, it is the default for a blank chip.

