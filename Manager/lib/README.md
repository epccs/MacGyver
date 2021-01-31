# Status Notes

```
timer_bsd timer setup with binary event timer from TCA0 that counts at F_CPU / (64 * 256)  [or F_CPU / (16 * 256) with slower F_CPU].
uart0_bsd init returns a pointer to FILE so redirect of stdin and stdout works (stdio.h streams)
twi0_bsd two ISR driven state machines, one for the master and another for the slave.
adc_bsd
```

# Quick Notes

adc_bsd: Interrupt-Driven Analog-to-Digital Converter

twi0_bsd: Interrupt-Driven Asynchronous I2C library almost like I have been using on m328pb and m324pb. Using lots of buffering.

uart0_bsd: Interrupt-Driven UART for AVR Standard IO facilities streams like I have been using on m328pb and m324pb.

timer_bsd: sets the first Timer A (TCA0) in split mode to give six 8-bit PWM channels (WO0..5) that do Single-Slope PWM Generation. The High Byte Timer Counter (TCAn.HCNT) is used to generate underflow events ("ticks") for timekeeping. The timekeeping count is continuous; it is not trying to count milliseconds. The Timer B hardware is clocked from CLK_TCA (e.g., same as TCA0) at  F_CPU = 16MHz it is 250kHz (e.g., 16000000/64), but the divider changes with selected clocks. I am hoping to use TCB for input capture, so these settings will need to be changed. Timer D is set up generically, but it is much too complicated to sort out at this point.

# Referance Materials

Spence Konde has started an Arduino board package repo; it is full of stuff that can give clues about how this hardware works.

https://github.com/SpenceKonde/DxCore

