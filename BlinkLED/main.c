/* Blink LED
Copyright (C) 2019 Ronald Sutherland

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

#include <stdbool.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/io.h>
#include "../lib/io_enum_bsd.h"
#include "../lib/timers_bsd.h"
#include "../lib/twi0_mc.h"

#define BLINK_DELAY 1000UL
static unsigned long blink_started_at;

// cycle the twi state machine on both the master and slave(s)
void i2c_ping(void)
{ 
    // ping I2C for an RPU bus manager 
    uint8_t i2c_address = 41; //I2C_ADDR_OF_BUS_MGR
    uint8_t data = 0;
    uint8_t length = 0;
    // uint8_t wait = 1;
    uint8_t sendStop = 1;
    for (uint8_t i =0;1; i++) // try a few times.
    {
        uint8_t twi_errorCode = TWI_MasterWrite(i2c_address, &data, length, sendStop); 
        if (twi_errorCode == 0) break; // ping was error free
        if (i>5) return; // give up after 5 trys
    }
    return; 
}

// don't block (e.g. _delay_ms(1000) ), ckeck if time has elapsed to toggle 
void blink(void)
{
    unsigned long kRuntime = elapsed(&blink_started_at);
    if ( kRuntime > BLINK_DELAY)
    {
        ioToggle(MCU_IO_AIN0);
        
        // next toggle 
        blink_started_at += BLINK_DELAY; 
    }
}

/*
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

*/



int main(void)
{
    ioCntl(MCU_IO_AIN0, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_AIN0, DIRECTION_OUTPUT);
    ioWrite(MCU_IO_AIN0,LOGIC_LEVEL_HIGH);

    // init i2c master with normal 100k clock
    // TWI_MasterInit(100000UL);

    //TCA0_HUNF used for timing, TCA0 split for 6 PWM's, TCB0..TCB2 set for three more PWM's.
    initTimers();

    // Enable global interrupts to start Timers and I2C
    sei();

    blink_started_at = milliseconds();

    while (1)
    {
        blink();
        // i2c_ping(); // That poor I2C address is going to lose it.
    }
}

