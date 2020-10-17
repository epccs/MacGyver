/*
Initialize AVR Timers and setup a tick timer for timekeeping
Copyright (c) 2020 Ronald S,. Sutherland

Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES 
WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE 
FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY 
DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, 
WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, 
ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

https://en.wikipedia.org/wiki/BSD_licenses#0-clause_license_(%22Zero_Clause_BSD%22)
*/

//#include <util/atomic.h>
#include <avr/interrupt.h>
#include "timers_bsd.h"

static unsigned long millisec = 0;

// pull in code for TIMERRTC
#ifdef USE_TIMERRTC_XTAL
#define USE_TIMERRTC
#endif

#ifndef USE_TIMERRTC
volatile unsigned long tick = 0;
static unsigned long millisec_tick_last_used = 0;
static uint16_t uS_balance = 0;
#else
volatile uint16_t tick = 0;
static uint16_t millisec_tick_last_used = 0;
#endif

#ifdef USE_TIMERA0
ISR(TCA0_HUNF_vect)
#elif defined(USE_TIMERRTC)
ISR(RTC_CNT_vect)
#else
  #error "no millisec timer selected"
#endif
{
    // swap to local since volatile has to be read from memory on every access
    uint32_t t = tick;
    ++t;
    tick = t;

#if defined(USE_TIMERA0)
    TCA0.SPLIT.INTFLAGS = TCA_SPLIT_HUNF_bm;
#elif defined(USE_TIMERRTC)
    RTC.INTFLAGS=RTC_OVF_bm;
#endif
}

/* Setup Timers, TCA0 is split into two 8 bit timers, the high underflow (HUNF) event it used for
   time tracking, it counts underflow events (not milliseconds). I need to work on ways to convert 
   milliseconds to the raw underflow count because I think that would work better than the milliseconds 
   function.
   Some F_CPU speeds do not track milliseconds perfectly with that function
*/
void initTimers()
{
#if (F_CPU == 24000000)
    // MICROSEC_TICK_CORRECTION 682.6666666666666 is not perfect
    _PROTECTED_WRITE(CLKCTRL_OSCHFCTRLA, CLKCTRL_FREQSEL_24M_gc);

#elif (F_CPU == 20000000)
    // MICROSEC_TICK_CORRECTION 819.2 is not perfect
    _PROTECTED_WRITE(CLKCTRL_OSCHFCTRLA, CLKCTRL_FREQSEL_20M_gc);

#elif (F_CPU == 16000000)
    // MICROSEC_TICK_CORRECTION 24
    _PROTECTED_WRITE(CLKCTRL_OSCHFCTRLA, CLKCTRL_FREQSEL_16M_gc);

#elif (F_CPU == 12000000)
    // MICROSEC_TICK_CORRECTION 365.3333333333333 is not perfect
    _PROTECTED_WRITE(CLKCTRL_OSCHFCTRLA, CLKCTRL_FREQSEL_12M_gc);

#elif (F_CPU == 8000000)
    // MICROSEC_TICK_CORRECTION 48.0
    _PROTECTED_WRITE(CLKCTRL_OSCHFCTRLA, CLKCTRL_FREQSEL_8M_gc);

#elif (F_CPU == 4000000)
    // MICROSEC_TICK_CORRECTION 24.0
    _PROTECTED_WRITE(CLKCTRL_OSCHFCTRLA, CLKCTRL_FREQSEL_4M_gc);

#elif (F_CPU == 2000000)
    // MICROSEC_TICK_CORRECTION 48.0
    _PROTECTED_WRITE(CLKCTRL_OSCHFCTRLA, CLKCTRL_FREQSEL_2M_gc);

#elif (F_CPU == 1000000)
    // MICROSEC_TICK_CORRECTION 48.0
    _PROTECTED_WRITE(CLKCTRL_OSCHFCTRLA, CLKCTRL_FREQSEL_1M_gc);
#else
    #error "F_CPU must be defined as a supported value"
#endif

    /* When shifting between Normal mode and Split mode, the functionality of some registers and bits change, but 
       their values do not. For this reason, disabling the peripheral (ENABLE = 0 in TCAn.CTRLA) and doing a hard Reset 
       (CMD = RESET in TCAn.CTRLESET) is recommended when changing the mode to avoid unexpected behavior.  */
    TCA0.SINGLE.CTRLA &= ~(TCA_SINGLE_ENABLE_bm);
    TCA0.SINGLE.CTRLESET = TCA_SINGLE_CMD_RESET_gc;

    /* Waveform Generation Mode 
       Single-slope PWM only (WGMODE = SINGLESLOPE in TCAn.CTRLB) in split mode */
    TCA0.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc;

    /* change to split mode */
    TCA0.SPLIT.CTRLD = TCA_SPLIT_SPLITM_bm;

    /* Period setting, 8-bit register in SPLIT mode which makes both OVF_LUNF and HUNF events */
    TCA0.SPLIT.LPER    = 0xFF; // the counter counts from 0 to 255 and then restarts from 0 and generates an "underflow" event
    TCA0.SPLIT.HPER    = 0xFF; // that is 256 counts, like a ATmega328p Timer0 set for fast PWM would do. I think they called it UNF because OVF is used in the 16 bit counter mode.

   /* Set the starting duty to 50%*/
    TCA0.SPLIT.LCMP0 = 0x7F;
    TCA0.SPLIT.LCMP1 = 0x7F;
    TCA0.SPLIT.LCMP2 = 0x7F;
    TCA0.SPLIT.HCMP0 = 0x7F;
    TCA0.SPLIT.HCMP1 = 0x7F;
    TCA0.SPLIT.HCMP2 = 0x7F;

    /* TCA Clock Select */
#if (F_CPU > 5000000)
    TCA0.SPLIT.CTRLA = (TCA_SPLIT_CLKSEL_DIV64_gc) | (TCA_SINGLE_ENABLE_bm);
#define MICROSEC_TICK_CORRECTION (( (64 * 256) / ( F_CPU / 1000000UL ) ) % 1000UL)
#define TICK_CORRECTION (( (64 * 256) / ( F_CPU / 1000UL ) ))
#define TIME_CORRECTION ( ( F_CPU ) / (64 * 256) )
#elif (F_CPU > 1000000)
    TCA0.SPLIT.CTRLA = (TCA_SPLIT_CLKSEL_DIV16_gc) | (TCA_SINGLE_ENABLE_bm);
#define MICROSEC_TICK_CORRECTION (( (16 * 256) / ( F_CPU / 1000000UL ) ) % 1000UL)
#define TICK_CORRECTION (( (16 * 256) / ( F_CPU / 1000UL ) ))
#define TIME_CORRECTION ( ( F_CPU ) / (16 * 256) )
#else
    TCA0.SPLIT.CTRLA = (TCA_SPLIT_CLKSEL_DIV8_gc) | (TCA_SINGLE_ENABLE_bm);
#define MICROSEC_TICK_CORRECTION (( (8 * 256) / ( F_CPU / 1000000UL ) ) % 1000UL)
#define TICK_CORRECTION (( (8 * 256) / ( F_CPU / 1000UL ) ))
#define TIME_CORRECTION ( ( F_CPU ) / (8 * 256) )
#endif

#ifdef TCA1
    TCA1.SINGLE.CTRLA &= ~(TCA_SINGLE_ENABLE_bm);
    TCA1.SINGLE.CTRLESET = TCA_SINGLE_CMD_RESET_gc;

    TCA1.SINGLE.CTRLB = TCA_SINGLE_WGMODE_SINGLESLOPE_gc;

    TCA1.SPLIT.CTRLD = TCA_SPLIT_SPLITM_bm;

    TCA1.SPLIT.LPER = 0xFF;
    TCA1.SPLIT.HPER = 0xFF;

    TCA1.SPLIT.LCMP0 = 0x7F;
    TCA1.SPLIT.LCMP1 = 0x7F;
    TCA1.SPLIT.LCMP2 = 0x7F;
    TCA1.SPLIT.HCMP0 = 0x7F;
    TCA1.SPLIT.HCMP1 = 0x7F;
    TCA1.SPLIT.HCMP2 = 0x7F;

    /* TCA Clock Select */
#if (F_CPU > 5000000)
    TCA1.SPLIT.CTRLA = (TCA_SPLIT_CLKSEL_DIV64_gc) | (TCA_SINGLE_ENABLE_bm);
#elif (F_CPU > 1000000)
    TCA1.SPLIT.CTRLA = (TCA_SPLIT_CLKSEL_DIV16_gc) | (TCA_SINGLE_ENABLE_bm);
#else
    TCA1.SPLIT.CTRLA = (TCA_SPLIT_CLKSEL_DIV8_gc) | (TCA_SINGLE_ENABLE_bm);
#endif
#endif

    /* TCA0 [and TCA1] Signals select */
    PORTMUX.TCAROUTEA = PORTMUX_TCA0_PORTC_gc
#if defined(TCA1)
                      | PORTMUX_TCA1_PORTB_gc
#endif
    ;

    /* TCB0..TCB2 [and TCB3..TCB4] Signals select. e.g., PWM is going to default pins*/
    PORTMUX.TCBROUTEA = PORTMUX_TCB0_DEFAULT_gc | PORTMUX_TCB1_DEFAULT_gc | PORTMUX_TCB2_DEFAULT_gc
#if defined(TCB3)
                        | PORTMUX_TCB3_DEFAULT_gc
#endif
#if defined(TCB4)
                        | PORTMUX_TCB4_DEFAULT_gc
#endif
    ;

#if defined(TCB4)
    TCB_t *timer_B_end = (TCB_t *)&TCB4;
#elif defined(TCB3)
    TCB_t *timer_B_end = (TCB_t *)&TCB3;
#else
    TCB_t *timer_B_end = (TCB_t *)&TCB2;
#endif

    // loop through all timers and set 8 bit PWM mode, 50% duty at enable, clock from TCA see TCA Clock Select above
    TCB_t *timer_B = (TCB_t *)&TCB0;
    do
    {
      timer_B->CTRLB = (TCB_CNTMODE_PWM8_gc);
      timer_B->CCMPL = 0xFE; // frequency
      timer_B->CCMPH = 0x80; // duty
      timer_B->CTRLA = (TCB_CLKSEL_TCA0_gc) | (TCB_ENABLE_bm);
      timer_B++;
    } while (timer_B <= timer_B_end);



#ifdef TCD0
    PORTMUX.TCDROUTEA = PORTMUX_TCD0_DEFAULT_gc;
    TCD0.CMPBCLR = 510; // 50% duty
    TCD0.CMPACLR = 510; // 50% duty
    TCD0.CTRLC = TCD_CMPDSEL_PWMB_gc |  TCD_CMPCSEL_PWMA_gc; // WOD gets PWM B, WOC gets PWM A
    TCD0.CTRLB = TCD_WGMODE_ONERAMP_gc;
    TCD0.CTRLA = (TCD_CNTPRES_DIV32_gc | TCD_SYNCPRES_DIV1_gc | TCD_CLKSEL_OSCHF_gc);
#endif

#ifdef USE_TIMERA0
    TCA0.SPLIT.INTCTRL |= TCA_SPLIT_HUNF_bm;
#elif defined(USE_TIMERRTC)
    while(RTC.STATUS); // PERBUSY flag in RTC.STATUS must be cleared
    RTC.PER=0xFFFF; // befor writing to this register to reset the RTC
#ifdef USE_TIMERRTC_XTAL
    _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA,0x03);
    RTC.CLKSEL=2; //external crystal
#else
    _PROTECTED_WRITE(CLKCTRL.OSC32KCTRLA,0x02);
    RTC.CLKSEL=0; // this is the power on value
#endif // USE_TIMERRTC_XTAL
    RTC.INTCTRL=0x01; //enable overflow interupt
    RTC.CTRLA=(RTC_RUNSTDBY_bm|RTC_RTCEN_bm|RTC_PRESCALER_DIV32_gc);//fire it up, prescale by 32.
#endif // USE_TIMERA0 or USE_TIMERRTC
}

// returns a count of Timer A underflow events.
// each tick is (64 * 256) = 16,384 crystal counts or 1.024mSec with F_CPU at 16MHz
unsigned long tickAtomic()
{
    unsigned long local;

    // an stomic transaction is done by turning off interrupts
    uint8_t oldSREG = SREG;
    cli();           // clear the global interrupt mask.
    local = tick;    // there are four bytes to copy but nothing can change at the moment
    SREG = oldSREG;  // restore global interrupt if they were enabled

    return local;
}

// convert up to 4 million milliseconds (1hr) to ticks
unsigned long cnvrt_milli(unsigned long millisec)
{
    unsigned long ticks = ( (millisec) * TIME_CORRECTION) / 1000;
    return ticks;
}

// convert time in milliseconds to ticks, slow and bulky
unsigned long cnvrt_milli_lrg(unsigned long m)
{
    uint64_t millisec = m;
    unsigned long ticks = (millisec * TIME_CORRECTION) / 1000;
    return ticks;
}

// return the elapsed ticks given a pointer to a past time
unsigned long elapsed(unsigned long *past)
{
    unsigned long now = tickAtomic();
    return now - *past;
}

// calculate milliseconds based on the TCA0_HUNF tick  
// call every 250 ticks or the time will not be correct.
// yes, this is tearable, stop tracking time in mSec, use tickAtomic, and a function to convert time into ticks (to do.)
unsigned long milliseconds(void)
{
    uint32_t now_tick;
    uint8_t status = SREG;
    cli();
        now_tick = tick; //tick is volatile so get a copy to work with
    SREG = status; // restore the Global Interrupt Enable Bit if it was set by sei()

    // differance between now and last tick used
    unsigned long ktick = now_tick - millisec_tick_last_used;
    uint8_t ktick_byt;
    if (ktick > 250) //limit looping delays
    {
         ktick_byt = 250;
    }
    else ktick_byt = (uint8_t) ktick;
    if (ktick_byt)
    {
        while( ktick_byt ) 
        {
            // update millisec time
            millisec += TICK_CORRECTION;
            ktick_byt--;
#ifdef USE_TIMERA0
            uS_balance  = uS_balance + MICROSEC_TICK_CORRECTION; // 1,2,4,8,16MHz have a perfect unit of time in microseconds, this has offset for others
            if (uS_balance > 1000) 
            {
                uS_balance = uS_balance - 1000;
                ++millisec; // add leap millisecond.
            }
#endif
        }
        millisec_tick_last_used = now_tick;
    }
    return millisec;
}


// after 2**32 counts of the tick value it will role over, e.g. 2**(14+32) crystal counts. 
// (2**(14+32))/16000000/3600/24 = 50.9 days

// Note a capture is 16 bits, and extending it has proven to be a problem. 
// it may be possible to merge the capture value and tick value to form a 46 bit event time. 
