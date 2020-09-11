#ifndef TimersTick_h
#define TimersTick_h

/* Timer tick opitons 
    USE_TIMERA0
    USE_TIMERRTC
    USE_TIMERRTC_XTAL
*/

#define USE_TIMERA0

extern void initTimers(void);
extern uint32_t tickAtomic(void);
extern unsigned long milliseconds(void);
unsigned long elapsed(unsigned long *);

#endif // TimersTick_h
