#pragma once

/* Timer tick opitons 
    USE_TIMERA0
    USE_TIMERRTC
    USE_TIMERRTC_XTAL
*/

#define USE_TIMERA0

extern void initTimers(void);
extern unsigned long tickAtomic(void);
extern unsigned long milliseconds(void); // not recomended
unsigned long elapsed(unsigned long *past);
unsigned long cnvrt_milli(unsigned long millisec);
unsigned long cnvrt_milli_lrg(unsigned long millisec);

