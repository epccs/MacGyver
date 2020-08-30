# Hardware

## Overview

This board connects a Pi Zero [W] to an AVR128DA28 by way of serial or UPDI.


## Inputs/Outputs/Functions

```
        TBD
```


## Uses

```
        TBD
```


## Notice

```
        TBD
 ```


# Table Of Contents

1. [Status](#status)
2. [Design](#design)
3. [Bill of Materials](#bill-of-materials)
4. [How To Use](#how-to-use)


# Status

![Status](./status_icon.png "Status")

```
        ^2  Done: 
            WIP: 
            Todo: Design, Layout, BOM, Review* (#= done), Order Boards, Assembly, Testing, Evaluation.
            *during review the Design may change without changing the revision.
            3-state fuffer? https://www.diodes.com/assets/product-showcases/Automotive-compliant-Single-Gate-LVC-Logic-Family.pdf
            AVR128DB has two power domains after a fuse is set, but the logistics of the fuse and keeping it fixed that way...

        ^1  Done: Design, Layout, BOM, Review* (#= done), Order Boards,
            WIP: Assembly, 
            Todo: Testing, Evaluation.
            *during review the Design may change without changing the revision.
            # swap the m4809 for a AVR128DA28
            # UART0 crossover connection to R-Pi
            (test) check if AREF can be pulled down by the pins push-pull hardware it is sharing (https://www.avrfreaks.net/comment/2922081)

            ^0  Done: Design, Layout, BOM, Review*, Order Boards,
            WIP: Assembly (will not do),
            Todo: Testing, Evaluation.
            *during review the Design may change without changing the revision.
            note this was not tested, and an error has been found with serial.
            # use Python https://github.com/mraardvark/pyupdi to upload AVR from R-Pi Zero
```

Debugging and fixing problems i.e. [Schooling](./Schooling/)

Setup and methods used for [Evaluation](./Evaluation/)


# Design

The board is 0.063 thick, FR4, two layer, 1 oz copper with ENIG (gold) finish.

![Top](./Documents/19260,Top.png "Top")
![TAssy](./Documents/19260,TAssy.jpg "Top Assy")
![Bottom](./Documents/19260,Bottom.png "Bottom")
![BAssy](./Documents/19260,BAssy.jpg "Bottom Assy")


## Electrical Schematic

![Schematic](./Documents/19260,Schematic.png "Schematic")

## Testing

Check correct assembly and function with [Testing](./Testing/)


# Bill of Materials

The BOM is a CVS file, import it into a spreadsheet program like LibreOffice Calc (or Excel), or use a text editor.

Option | BOM's included
----- | ----- 
A. | [BRD] 
M. | [BRD] [SMD] [HDR] 
N. | [BRD] [SMD] [HDR] [POL] [PLUG]

[BRD]: ./Design/19260BRD,BOM.csv
[HDR]: ./Design/19260HDR,BOM.csv
[PLUG]: ./Design/19260PLUG,BOM.csv
[POL]: ./Design/19260POL,BOM.csv
[SMD]: ./Design/19260SMD,BOM.csv

[Order Form](https://rpubus.org/Order_Form.html)


# How To Use

Your [Raspberry Pi] is your computer, you are the expert because I am not. Don't buy this board and expect that I can help with your computer, I can not. 

[Raspberry Pi]: https://www.raspberrypi.org/forums/

## Pi Zero Setup 

The Pi Zero is a Single Board Computer (SBC) running [Linux]. I use it as a network machine and to run a toolchain at the network edge. It has enough memory and processing power for the AVR toolchain (and others that I have not tested). It also does self-hosted compiling (e.g., compiles programs to run on itself) and has lots of applications and services. My use is sort of like a headless test bench computer embedded next to the bare metal control boards. It is a classic control system with a host and instruments on a communication bus (hard-link, e.g., GPIB and ilk), but it is headless, so I interact with the target over an SSH session where I run programs that operate the hard-link. 

[Linux]: https://github.com/epccs/RPUpi/blob/master/Hardware/Testing/linux.md

The BCM2835 Broadcom chip used in the Raspberry Pi Zero is an ARM11 running at 1 GHz it has support with the [Raspbian] distribution. 

[Raspbian]: https://www.raspbian.org/


## Serial

The Pi serial port (RX is BCM 15 and TX is BCM 14) is crossover connected to the AVR128DA28 serial. 

![Pi Pinout](https://raw.githubusercontent.com/epccs/RPUpi/master/Hardware/Documents/Pi-pinout-graphic.png)

UPDI does not use the nRTS/nCTS pins (16/17).

Note: version ^0 did not crossover.
