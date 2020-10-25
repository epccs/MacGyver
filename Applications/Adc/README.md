# Analog-to-Digital Converter

## Todo

The referances are going to be held on the manager, not the application eeprom.

## Overview

Adc is an interactive command line program that demonstrates control of an Analog-to-Digital Converter. 

A customized library routine is used to operate the AVR's ADC, it has an ISR that is started with the enable_ADC_auto_conversion function to read the channels one after the next in a burst. In this case, the loop starts the burst at timed intervals. The ADC clock runs at 1MegHz (must be > 150kHz) and it takes about 36 (see "Conversion Timing" in DS is 20, and delay 16 to settle referance) cycles to do the conversion, thus a burst takes over (ISR overhead) .29 milliseconds (e.g. 8*36*(1/1000000)) to scan eight channels. The ADC is turned off after each burst unless free running is set, which would automaticly start the next burst.

The channel number is used in a switch statement (see LoadAnalogCal() function in ../lib/references.c) to set the channel configuraiton, reference, and calibration.


# Manager has Reference and Callibration Values

The application controller and manager have a private I2C bus between them.

```
Callibraion         type        i2c command/select/data         manager defaults 
alt_i               INT16       32,0,0                          32,(12bits from adc ch 0)
alt_v               INT16       32,0,1                          32,(12bits from adc ch 1)
pwr_i               INT16       32,0,2                          32,(12bits from adc ch 6)
pwr_v               INT16       32,0,3                          32,(12bits from adc ch 7)
ref_extern_vdd      FLOAT       38,0,0,0,0,0                    38,0,0x40,0xA0,0x0,0x0
ref_intern_1v0      FLOAT       38,1,0,0,0,0                    38,1,0x3F,0x8A,0x3D,0x71
ref_intern_2v0      FLOAT       38,2,0,0,0,0                    38,2,TBD
ref_intern_4v1      FLOAT       38,3,0,0,0,0                    38,3,TBD
alt_i_callibraion   FLOAT       33,0,0,0,0,0                    33,0,0x3A,0x8E,0x38,0xE4
alt_v_callibraion   FLOAT       33,1,0,0,0,0                    33,1,0x3C,0x30,0x0,0x0
pwr_i_callibraion   FLOAT       33,2,0x39,0x96,0x96,0x96        33,2,0x39,0x96,0x96,0x96
pwr_v_callibraion   FLOAT       33,3,0x3B,0xEA,0x88,0x1A        33,3,0x3B,0xEA,0x88,0x1A
```

The ref_extern_vdd value should be used when VREF_REFSEL_VDD_gc is selected. The DACREF0..2 can be connected to positive ADC input to directly measure the 1V024, 2V048, and 4V096 referances that go to the DAC. I am ignoring the 2V5 referance for now.

The internal bandgaps should be within +/-4% from the manufacturer, but are temperature stable. Once known, they are useful references.

[SelfTest] will load calibration values on the manager for references.

[SelfTest]: https://github.com/epccs/MacGyver/tree/master/Applications/SelfTest

# Firmware Upload

The manager connected to the host that uploads needs to be advised what bootload address will receive the upload. I do this with some commands on the SMBus interface from a Raspberry Pi single-board computer.

``` 
sudo apt-get install i2c-tools python3-smbus
sudo usermod -a -G i2c your_user_name
# logout for the change to take
python3
import smbus
bus = smbus.SMBus(1)
#write_i2c_block_data(I2C_ADDR, I2C_COMMAND, DATA)
#read_i2c_block_data(I2C_ADDR, I2C_COMMAND, NUM_OF_BYTES)
# what is my address
bus.write_i2c_block_data(42, 0, [255])
print("'"+chr(bus.read_i2c_block_data(42, 0, 2)[1])+"'" )
'2'
# I want to bootload address '1'
bus.write_i2c_block_data(42, 3, [ord('1')])
print("'"+chr(bus.read_i2c_block_data(42, 3, 2)[1])+"'" )
'1'
# clear the host lockout status bit so nRTS from my R-Pi on '2' will triger the bootloader on '1'
bus.write_i2c_block_data(42, 7, [0])
print(bus.read_i2c_block_data(42,7, 2))
[7, 0]
exit()
```

Now the serial port connection (see BOOTLOAD_PORT in Makefile) can reset the MCU and execute the receiving bootloader (optiboot) so that the 'make bootload' rule can upload a new binary image in the application area of the MCU's flash memory.

``` 
sudo apt-get install make git picocom gcc-avr binutils-avr gdb-avr avr-libc avrdude
git clone https://github.com/epccs/Gravimetric/
cd /Gravimetric/Applications/Adc
make
make bootload
...
avrdude done.  Thank you.
``` 

Now connect with picocom (or ilk).


``` 
#exit is C-a, C-x
picocom -b 38400 /dev/ttyAMA0
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
/1/id?
{"id":{"name":"Adc","desc":"Gravimetric (17341^1) Board /w ATmega324pb","avr-gcc":"5.4.0"}}
```

##  /0/analog? 0..7\[,0..7\[,0..7\[,0..7\[,0..7\]\]\]\]    

Analog-to-Digital Converter reading from up to 5 channels. The reading repeats every 2 Seconds until the Rx buffer gets a character. Channel 11 is the input voltage (PWR_V), channel 10 is the input current (PWR_I), channel 8 is the alternate input current (ALT_I), channel 9 is the alternate input voltage (ALT_V), channels 0..3 can read up to about 4.5V (higher voltages are blocked by a level shift). Channel 4..7 are from floating test points. Channels 8..11 are from the manager over a private I2C connection. 

``` 
/1/analog? 8,9,10,11
{"ALT_I":"0.00","ALT_V":"0.00","PWR_I":"0.02","PWR_V":"12.81"}
/1/analog? 0,1,2,3,4
{"ADC0":"3.75","ADC1":"3.77","ADC2":"3.77","ADC3":"3.77","ADC4":"3.77"}
```

Channels 0..4 were floating when I ran the above.

Channels 8..11 are taken from the manager over the I2C interface.


##  /0/avcc 4500000..5500000

Calibrate the AVCC reference in microvolts.

``` 
# do this with the SelfTest
/1/avcc 4958500
{"REF":{"extern_avcc":"4.9585",}}
``` 


##  /0/onevone 900000..1300000

Set the 1V1 internal bandgap reference in microvolts.

```
# do this with the SelfTest
/1/onevone 1100000
{"REF":{"intern_1v1":"1.1000",}}
``` 

The SelfTest will calculate this value when it is ran based on the AVCC value compiled into source.


##  /0/reftoee

Save the reference in static memory to EEPROM.

```
# do this with the SelfTest
/1/reftoee
{"REF":{"extern_avcc":"4.9585","intern_1v1":"1.1000",}}
```


##  /0/reffrmee

Load the reference from EEPROM into static memory.

```
/1/reffrmee
{"REF":{"extern_avcc":"4.9785","intern_1v1":"1.0858",}}
```

