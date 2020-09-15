# BlinkLED

## Overview

TBD


## Firmware Upload

Use a UPDI upload tool. Run 'make' to compile and 'make updi' to upload.

```
sudo apt-get install make git gcc-avr binutils-avr gdb-avr avr-libc
git clone https://github.com/mraardvark/pyupdi
git clone https://github.com/epccs/MacGyver/
cd /MacGyver/BlinkLED
make all
make updi
```

## How To Use

Kicking the tires

Hopefully, the firmware upload worked; it should have switched the serial port connection back to UART mode so I can use picocom to interact with the AVR's USART0.

``` 
[picocom -b 38400 /dev/ttyUSB0]
picocom -b 38400 /dev/ttyAMA0
...
a
# exit is C-a, C-x
# if it aborts you can reset with pyupdi
make reset
``` 

An 'a' will toggle the blinking (off), and another will toggle it on; after a few toggles, it will abort.

The AVR128DA28 starts running at 24MHz/6 (4MHz) from the factory. To run at another frequency, we can change the clock select bit field (e.g., CLKCTRL_FREQSEL_16M_gc). The timers_bsd.c will select the correct option based on the F_CPU value that the Makefile passes to the compiler during the build.

To set the clock with code requires meeting a 4 cycle timing.

```
// The magic is found in xmega.h and its use of consecutive OUT then STS in this sequence guarantee the timing.

/*
#define _PROTECTED_WRITE(reg, value)				\
  __asm__ __volatile__("out %[ccp], %[ccp_ioreg]" "\n\t"	\
		       "sts %[ioreg], %[val]"			\
		       :					\
		       : [ccp] "I" (_SFR_IO_ADDR(CCP)),		\
			 [ccp_ioreg] "d" ((uint8_t)CCP_IOREG_gc),	\
			 [ioreg] "n" (_SFR_MEM_ADDR(reg)),	\
			 [val] "r" ((uint8_t)value))
*/

// set clock to 20MHz (needs 5V on supply to function)
void clock_init(void) {
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_1X_gc | CLKCTRL_PEN_bm);
}

```

This info was from 

https://www.avrfreaks.net/forum/tutsoft-avr-01-series-setting-tca0-split-mode-pwm