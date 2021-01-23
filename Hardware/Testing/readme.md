# Description

This is a list of Test preformed on each MacGyver after assembly.

# Table of References


# Table Of Contents:

1. Basics
2. Assembly check
3. IC Solder Test
4. Power Protection
5. Power Without SMPS

4. Bias +5V
5. Start Latch
6. Programing Over UPDI With USB Bridge


## Basics

These tests are for an SMD assembled MacGyver board 19260^0 which may be referred to as a Unit Under Test (UUT). If the UUT fails and can be reworked then do so, otherwise it needs to be scraped. 

**Warning: never use a soldering iron to rework ceramic capacitors due to the thermal shock.**
    
Items used for test.

![ItemsUsedForTest](./19260_ItemsUsedForTest.jpg "Items Used For Test")


## Assembly check

After assembly check the circuit carefully to make sure all parts are soldered and correct. The device marking is used as the part name on the schematic and assembly drawing. Check the bill of materials to figure out what the device is.


## IC Solder Test

Check continuity between pin and pad by measuring the reverse body diode drop from 0V (aka ground) and all other IC pads not connected to 0V. This value will vary somewhat depending on what the pin does, but there is typically an ESD diode to ground or sometimes a body diode (e.g. open drain MOSFET), thus a value of .4V to .7V is valid to indicate a solder connection.


## Power Protection

Apply a current limited (20mA) supply set with 5V to the PWR and 0V connector J8 pin 1 and pin 2 in reverse and verify that the voltage does not get through. Adjust the supply to 36V and verify no current is passing.

Apply a current limited (20mA) supply set with 5V to the ALT and 0V connector J8 pin 3 and pin 2 in reverse and verify that the input is 100k Ohm (e.g., 0.36mA@36V). Adjust the supply to 36V and verify.


## Power Without SMPS

Apply a current limited (20mA) supply set with 7V to the PWR and 0V connector J7 and verify that the voltage does get through. Adjust the supply so the LED is on and stable and measure voltage, adjust supply to 30V measure input current. 

NOTE for referance the zener voltage on Q5 is 7.75V at 30V.

```
{ "LEDON_V":[10.6,10.6,10.7,],
  "PWR@7V_mA":[0.125,0.07,0.061,],
  "PWR@30V_mA":[1.5,1.7,1.1,] }
```


## Bias +5V

Apply a 30mA current limited 5V source to +5V (U1 pin 3). Check that the input current is for a blank MCU. Turn off the power.

```
{ "I_IN_BLANKMCU_mA":[3.0,]}
```

Note: The AVR128DA starts running at 3.33MHz (20MHz/6) from the factory. A fuse (OSCCFG) can select a 16MHz clock. Prescale of the clock is done at runtime.


## Start Latch

The RPUusb has a jumper (R3) that needs shorted so that USB power can reach the UUT. Do not plug in the USB cable yet. The RPUusb will act as slight load to test the start latch while +5V is applied to whre U1 would send power. Apply a 30mA current limited 5V source to +5V (U1 pin 3). Press the start button.

```
{ "I_IN_LATCHED_mA":[13.7,]}
```

## Programing Over UPDI With USB Bridge

TBD



