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



## Basics

These tests are for an SMD assembled MacGyver board 19260^2 which may be referred to as a Unit Under Test (UUT). If the UUT fails and can be reworked then do so, otherwise it needs to be scraped. 

**Warning: never use a soldering iron to rework ceramic capacitors due to the thermal shock.**
    
Items used for test.

![ItemsUsedForTest](./19260_ItemsUsedForTest.jpg "Items Used For Test")


## Assembly check

After assembly check the circuit carefully to make sure all parts are soldered and correct. The device marking is used as the part name on the schematic and assembly drawing. Check the bill of materials to figure out what the device is.


## IC Solder Test

Check continuity between pin and pad by measuring the reverse body diode drop from 0V (aka ground) and all other IC pads not connected to 0V. This value will vary somewhat depending on what the pin does, but there is typically an ESD diode to ground or sometimes a body diode (e.g. open drain MOSFET), thus a value of .4V to .7V is valid to indicate a solder connection.


## Power Protection

Apply a current limited (20mA) supply set with 5V to the PWR and 0V connector J6 pin 1 and pin 2 in reverse and verify that the voltage does not get through. Adjust the supply to 36V and verify no current is passing.

Apply a current limited (20mA) supply set with 5V to the ALT and 0V connector J6 pin 3 and pin 2 in reverse and verify that the input is 100k Ohm (e.g., 0.36mA@36V). Adjust the supply to 36V and verify.


## Power Without SMPS

Apply a current limited (20mA) supply set with 7V to the PWR and 0V connector J6 and verify that the voltage does get through. Adjust the supply so the LED is on and stable and measure voltage, adjust supply to 30V measure input current. 

NOTE for referance the zener voltage on Q9 is 7.75V at 30V.

```
{ "LEDON_V":[10.7,],
  "PWR@7V_mA":[0.075,],
  "PWR@30V_mA":[1.29,] }
```


## Bias +5V

Apply a 30mA current limited 5V source to +5V to J1 pin 1 and 0V to J1 pin 2. Check that the input current is for a blank MCU. Turn off the power.

```
{ "I_IN_BLANKMCU_mA":[2.0,]}
```

Note: The AVR128DA is not in its socket and AVR128DB starts running at 4MHz from the factory. Selecting the other clocks is done at runtime.


## Install SMPS

Add U3 to the board now.

With U3 installed measure its output voltage between J1 pin 2 and pin 1, while the supply is set at 12.8V and a 30mA current limit on PWR connector J6 pin 1 and 0V on pin 2, also measure the supply input current.

```
{ "+5V_V":[4.9655,],
   "Iin@12V8_V":[2.3,]}
```

Note: The AVR128DA is not in its socket and AVR128DB starts running at 4MHz from the factory.

************ TBD

## Self Test


