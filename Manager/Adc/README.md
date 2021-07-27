# Analog-to-Digital Converter

## Todo

(done) show ADC values in debug
(done, ouch)verify values are correct
add way to return ADC values on TWI
add references and return on TWI
add corrections and return on TWI

## Overview

The manager can read input voltage (PWR_V), input current (PWR_I), charger voltage (ALT_V), and charger current (ALT_I). This program will hopfuly demenstrate how those values can be pushed to the application over the I2C (multi-master) interface.

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
picocom -b 38400 /dev/ttyUSB2
```

or log the terminal session

```bash
script -f -c "picocom -b 38400 /dev/ttyUSB2" stuff.log
```

values look like

```json
{"ADC1":"1431","ADC2":"3","ADC3":"34","ADC4":"1432"}
{"ADC1":"1431","ADC2":"3","ADC3":"34","ADC4":"1432"}
{"ADC1":"1432","ADC2":"3","ADC3":"33","ADC4":"1432"}
{"ADC1":"1431","ADC2":"3","ADC3":"32","ADC4":"1431"}
```

But first reading is wrong. ADC1 is ALT_I and should be near zero. ADC2 is ALT_V and should also be near 0, so that checks. ADC3 is PWR_I and has .043V so 34*5.0/2**12 checks. ADC4 is PWR_V and has 1.746V so 1432*5.0/2**12 checks.

Welp, I had the mux set to read from AIN0 when channel one got set up; these values are reasonable. The folloing is done with adcSingle() which blocks.

```json
{"ADC1":"3","ADC2":"0","ADC3":"36","ADC4":"1431"}
{"ADC1":"4","ADC2":"0","ADC3":"32","ADC4":"1431"}
{"ADC1":"4","ADC2":"0","ADC3":"32","ADC4":"1431"}
{"ADC1":"3","ADC2":"0","ADC3":"35","ADC4":"1431"}
{"ADC1":"3","ADC2":"0","ADC3":"35","ADC4":"1431"}
```

LOL now I can't get the burst working, what did I do, need timeout... Turned out to be the adc_isr_status test (which is now removed), reading from adcAtomic() which is done with an adc_burst are the same as adcSingle() above.
