# Status Notes


```
timer_bsd works at 1,2,4,8, and 16MHz... but has issue at other speeds.


```

# Quick Notes

timer_bsd: sets the first Timer A (TCA0) in split mode to give six 8-bit PWM channels (WO0..5) that do Single-Slope PWM Generation. The High Byte Timer Counter (TCAn.HCNT) is used to generate underflow events ("ticks") for timekeeping. The timekeeping count is continuous; it is not trying to count milliseconds. The Timer B hardware is clocked from CLK_TCA (e.g., same as TCA0) at  F_CPU = 16MHz it is 250kHz (e.g., 16000000/64), but the divider changes with selected clocks. I am hoping to use TCB for input capture, so these settings will need to be changed. Timer D is set up generically, but it is much too complicated to sort out at this point.

# Referance Materials

Spence Konde has started an Arduino board package repo; it is full of stuff that can give clues about how this hardware works.

https://github.com/SpenceKonde/DxCore