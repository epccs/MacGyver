# BlinkLED

## Overview

TBD


## Firmware Upload

Use a UPDI upload tool. Run 'make' to compile and 'make updi' to upload.

```
sudo apt-get install make git gcc-avr binutils-avr gdb-avr avr-libc
git clone https://github.com/mraardvark/pyupdi
git clone https://github.com/epccs/PiUpdi/
cd /PiUpdi/BlinkLED
make
make updi
...
TBD
```

## How To Use

Kicking the tires

The 4809 starts running at 20MHz/6 (3.33MHz) from factory, so to run at other frequancy we need to change the divider or select the other clock. 

A fuse (OSCCFG) can be set to select a 16MHz base clock, but the factory default is 20MHz.

To set the clock with code requires meeting a 4 cycle timing requirement.

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