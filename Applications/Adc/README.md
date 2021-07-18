# Analog-to-Digital Converter

## Todo

rpu address is set to '0' since rpu_mgr.c needs updates,
The referances calibration are going to be held on the manager.

## Overview

Adc is an interactive command line program that demonstrates control of an Analog-to-Digital Converter.

A customized library routine is used to operate the AVR's ADC, it has an ISR that is started with the enable_ADC_auto_conversion function to read the channels one after the next in a burst. In this case, the loop starts the burst at timed intervals. The ADC clock runs at 1MegHz (must be > 150kHz) and it takes about 36 (see "Conversion Timing" in DS is 20, and delay 16 to settle referance) cycles to do the conversion, thus a burst takes over (ISR overhead) .29 milliseconds (e.g. 8*36*(1/1000000)) to scan eight channels. The ADC conversions stop after each burst unless free running is set, which would automaticly start the next burst.

The channel number is used in a switch statement (see LoadAdcConfig() function in ../lib/references.c) to set the channel configuraiton, reference, and calibration.

## Manager has Reference and Callibration Values

The application controller and manager will have a private I2C bus between them.

```text
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

## Firmware Upload

TBD, but at this time the updi rule needs some python scripts in RPUusb

```bash
sudo apt-get install make git picocom gcc-avr binutils-avr gdb-avr avr-libc avrdude
git clone https://github.com/epccs/RPUusb/
git clone https://github.com/epccs/MacGyver/
cd /MacGyver/Applications/Adc
make all
make updi
...
avrdude done.  Thank you.
```

Now connect with picocom (or ilk).

```bash
#exit is C-a, C-x
picocom -b 38400 /dev/ttyUSB0
#picocom -b 38400 /dev/ttyAMA0
```

or log the terminal session

```bash
script -f -c "picocom -b 38400 /dev/ttyUSB0" stuff.log
```

## Commands

Commands are interactive over the serial interface at 38400 baud rate. The echo will start after the second character of a new line.

## /\[rpu_address\]/\[command \[arg\]\]

rpu_address is taken from the manager (e.g. get_Rpu_address() which is included form ../lib/rpu_mgr.h). The value of rpu_address is used as a character in a string, which means don't use a null value (C strings are null terminated) as an adddress. The ASCII value for '1' (0x31 or 49) is easy and looks nice.

Commands and their arguments follow.

## /0/id? \[name|desc|avr-gcc\]

identify

```bash
/1/id?
{"id":{"name":"Adc","desc":"MacGyver (19260^1) Board /w AVR128DA28","avr-gcc":"5.4.0"}}
```

## /0/analog? 0..7\[,0..7\[,0..7\[,0..7\[,0..7\]\]\]\]

Analog-to-Digital Converter reading from up to 5 channels. The reading repeats every 2 Seconds until the Rx buffer gets a character. Channels 0..7 can read up to the VDD voltage, they are [floating] inputs unless connected to somthing.

[floating]: https://learn.adafruit.com/circuit-playground-digital-input/floating-inputs

```bash
/0/analog? 0,1,2,3,4
{"ADC0":"4.9988","ADC1":"3.4302","ADC2":"2.9431","ADC3":"2.5159","ADC4":"2.1887"}
{"ADC0":"4.9988","ADC1":"3.4290","ADC2":"2.9395","ADC3":"2.4963","ADC4":"2.2607"}
{"ADC0":"4.9988","ADC1":"3.4314","ADC2":"2.9431","ADC3":"2.5000","ADC4":"2.1570"}
{"ADC0":"4.9988","ADC1":"3.4314","ADC2":"2.9431","ADC3":"2.5159","ADC4":"2.1863"}
{"ADC0":"4.9988","ADC1":"3.4363","ADC2":"2.9590","ADC3":"2.5720","ADC4":"2.2583"}
```

ADC0 has 5V on it, the others are floating.

```bash
/0/adc? 0,1,2,3,4
{"ADC0":"4095","ADC1":"2733","ADC2":"2382","ADC3":"2100","ADC4":"1878"}
{"ADC0":"4095","ADC1":"2775","ADC2":"2395","ADC3":"2067","ADC4":"1907"}
{"ADC0":"4095","ADC1":"2791","ADC2":"2405","ADC3":"2064","ADC4":"1911"}
{"ADC0":"4094","ADC1":"2796","ADC2":"2409","ADC3":"2064","ADC4":"1913"}
{"ADC0":"4095","ADC1":"2798","ADC2":"2408","ADC3":"2064","ADC4":"1913"}
{"ADC0":"4094","ADC1":"2799","ADC2":"2408","ADC3":"2062","ADC4":"1913"}
{"ADC0":"4094","ADC1":"2800","ADC2":"2407","ADC3":"2059","ADC4":"1909"}
{"ADC0":"4095","ADC1":"2802","ADC2":"2408","ADC3":"2059","ADC4":"1909"}
{"ADC0":"4095","ADC1":"2802","ADC2":"2410","ADC3":"2058","ADC4":"1907"}
```
