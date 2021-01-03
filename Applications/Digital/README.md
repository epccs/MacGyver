# Digital Input/Output

## Overview

Digital is an interactive command line program that demonstrates control of the digital input/output from AVR128DA28 pins PD0..PD7 (AIN0..AIN7).


## ToDo

Add Control so that digital Input Buffer can be enabled/disabled and pin used for ADC, which would allow ADC to use the pin. 
The I2C and UART0 pins may not be used for general purpose I/O.
SPI, Uart1 and Uart2 pins might be added at some point. 



## Firmware Upload

TBD, but at this time the updi rule needs some python scripts in RPUusb

``` 
sudo apt-get install make git picocom gcc-avr binutils-avr gdb-avr avr-libc avrdude
git clone https://github.com/epccs/RPUusb/
git clone https://github.com/epccs/MacGyver/
cd /MacGyver/Applications/Digital
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

rpu_address is taken from the I2C address 0x29 (e.g. ../Uart/id.h get_Rpu_address() ). The value of rpu_address is used as a character in a string, which means don't use a null value (C strings are null terminated), but the ASCII value for '1' (0x31) is easy and looks nice, though I fear it will cause some confusion when it is discovered that the actual address value is 49.

The STATUS_LED is bliked fast (twice per second) if the I2C address is not found, also the rpu_address defaults to '0'. 

Commands and their arguments follow.


## /0/id? \[name|desc|avr-gcc\]

Identify is from ../Uart/id.h Id().

``` 
/0/id?
{"id":{"name":"Digital","desc":"MacGyver (19260^1) Board /w AVR128DA28","avr-gcc":"5.4.0"}}
```

##  /0/iodir 0..7,INPUT|OUTPUT

The AVR128DA has control registers that can Set (DIRSET) or Clear (DIRCLR) the direction contorl bits with a hardware method (that is also atomic). This is a more obious* way to set/clear bits, it is how most modern MCU (ARM, RISCV...) do things.

``` 
/0/iodir 0,OUTPUT
{"AIN0":"OUTPUT"}
/0/iodir 1,INPUT
{"AIN1":"INPUT"}
```

* CBI and SBI instructions operate on the lower 32 I/O bytes of memory (don't confuse those with CBR and SBR that work on the 32 registers, e.g., SREG). If a DDR is in that range, then those instructions are needed, but outside that range, others are used. The need for CBI and SBI is removed, and the resulting executable does not depend on where the memory is it is working with; thus, it is more obvious.


##  /0/iowrt 0..7,HIGH|LOW    

The AVR128DA has control registers that can Set (OUTSET) or Clear (OUTCLR) the output control bits with a hardware method (and is also atomic). 

``` 
/0/iowrt 0,LOW
{"AIN0":"LOW"}
/0/iowrt 1,HIGH
{"AIN1":"LOW"}
```

AIN1 is set as INPUT so it is not in the push-pull mode. Note a HIGH does not turn on the pull-up.


##  /0/iotog 0..7

The AVR128DA has a control register that can toggle (OUTTGL) the output control bits with a hardware method (and is also atomic). 

``` 
/0/iotog 0
{"AIN0":"HIGH"}
/0/iotog 0
{"AIN0":"LOW"}
```

The current rating of the AVR128DA push-pull is incredible, but you should read the datasheet for yourself.

https://ww1.microchip.com/downloads/en/DeviceDoc/AVR128DA28-32-48-64-DataSheet-DS40002183B.pdf


##  /0/iore? 0..7

Read the Port Input Register (PINx) bit that was latched during last low edge of the system clock.

``` 
/0/iord? 1
{"AIN1":"LOW"}
/0/iord? 0
{"AIN0":"LOW"}
```
