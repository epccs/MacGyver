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

Clock routing example

``` C
// test program, routing external 32kHz crystal -> RTC (on PA2) -> TCA0 (on PA0) -> TCB1 (on PA3)
// from El Tangas post #40
// https://www.avrfreaks.net/forum/mega4809-tca-oddity

void set_clock(){
    _PROTECTED_WRITE(CLKCTRL.XOSC32KCTRLA, CLKCTRL_RUNSTDBY_bm | CLKCTRL_ENABLE_bm);      // Enable 32kHz crystal; run in standby is needed.
    _PROTECTED_WRITE(CLKCTRL.MCLKCTRLB, CLKCTRL_PDIV_2X_gc | CLKCTRL_PEN_bm);             // Prescaler.
}

void RTC_init(){
    // The RTC is programmed to generate an event every other clock cycle, clocked by a 32kHz crystal.
    // Since each event pulse takes one RTC cycle, this means a square wave of 16kHz is generated on the RTC event output.
    RTC.CLKSEL = RTC_CLKSEL_TOSC32K_gc;
    RTC.PER = 1;
    RTC.CTRLA = RTC_PRESCALER_DIV1_gc | RTC_RTCEN_bm;
}

void EVSYS_init(){
    // Route the 16kHz square wave from RTC to both EVOUTA (PA2), for observation, and to the TCA event input, to clock TCA.
    EVSYS.CHANNEL0 = EVSYS_GENERATOR_RTC_OVF_gc;
    EVSYS.USEREVOUTA = EVSYS_CHANNEL_CHANNEL0_gc;       // Port direction is overridden automatically
    EVSYS.USERTCA0 = EVSYS_CHANNEL_CHANNEL0_gc;
}

void TCA_init(){
    // Generate square wave at TCA clock /2 on TCA output 0 (PA0).
    TCA0.SINGLE.CTRLB = TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc;
    TCA0.SINGLE.CMP0 = 1;
    TCA0.SINGLE.PER = 1;
    PORTA.DIRSET = 1 << 0;      // Set PA0 as output. This is not automatic for TCA.

    // This basically sets the TCA event input as clock.
    // Both edges clock TCA, this allows the 16kHz square wave coming from RTC to clock TCA @32kHz, thus recovering the full crystal clock.
    TCA0.SINGLE.EVCTRL = TCA_SINGLE_EVACT_ANYEDGE_gc | TCA_SINGLE_CNTEI_bm;

    // Enable TCA. The divisor seems to be ignored, the clock is set by the event input.
    TCA0.SINGLE.CTRLA = TCA_SINGLE_CLKSEL_DIV1_gc | TCA_SINGLE_ENABLE_bm;
}

void TCB_init(){
    // Generate square wave at TCB clock /2 on TCB1 output (PA3).
    // Enabling TCB output also automatically overrides port direction, unlike TCA.
    TCB1.CTRLB = TCB_CCMPEN_bm | TCB_CNTMODE_PWM8_gc;
    TCB1.CCMPL = 1;
    TCB1.CCMPH = 1;

    // Clock TCB from TCA, therefore indirectly from the 32kHz crystal.
    TCB1.CTRLA = TCB_ENABLE_bm | TCB_CLKSEL_CLKTCA_gc;
}

```