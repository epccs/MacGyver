/*
Adc is a command line controled demonstration of Interrupt Driven Analog Conversion
Copyright (C) 2016 Ronald Sutherland

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
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include "../lib/timers_bsd.h"
#include "../lib/uart1_bsd.h"
#include "../lib/adc_bsd.h"
#include "../lib/twi.h"
#include "../lib/io_enum_bsd.h"
#include "analog.h"

#define ADC_DELAY_MILSEC 200UL
static unsigned long adc_started_at;
static unsigned long adc_delay_milsec;

#define BLINK_DELAY 1000UL
static unsigned long blink_started_at;
static unsigned long blink_delay;

static int got_a;
FILE *uart1;

/*
void ProcessCmd()
{ 
    if ( (strcmp_P( command, PSTR("/id?")) == 0) && ( (arg_count == 0) || (arg_count == 1)) )
    {
        Id("Adc");
    }
    if ( (strcmp_P( command, PSTR("/analog?")) == 0) && ( (arg_count >= 1 ) && (arg_count <= 5) ) )
    {
        Analogf(cnvrt_milli(2000UL)); // update every 2 sec until terminated
    }
    if ( (strcmp_P( command, PSTR("/adc?")) == 0) && ( (arg_count >= 1 ) && (arg_count <= 5) ) )
    {
        Analogd(cnvrt_milli(2000UL)); // update every 2 sec until terminated
    }
}
*/

void setup(void) 
{
    // To reduce power consumption, the digital input buffer has to be disabled on the pins used as inputs for ADC. 
    // This is configured by the I/O Pin Controller (PORT).
    ioDir(MCU_IO_ALT_I, DIRECTION_INPUT);
    ioCntl(MCU_IO_ALT_I, PORT_ISC_INPUT_DISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_ALT_V, DIRECTION_INPUT);
    ioCntl(MCU_IO_ALT_V, PORT_ISC_INPUT_DISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_PWR_I, DIRECTION_INPUT);
    ioCntl(MCU_IO_PWR_I, PORT_ISC_INPUT_DISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_PWR_V, DIRECTION_INPUT);
    ioCntl(MCU_IO_PWR_V, PORT_ISC_INPUT_DISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);

    // STATUS_LED
    ioCntl(MCU_IO_MGR_LED, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_MGR_LED, DIRECTION_OUTPUT);
    ioWrite(MCU_IO_MGR_LED,LOGIC_LEVEL_HIGH);
    
    // TCA0_HUNF used for timing, TCA0 split for 6 PWM's.
    initTimers();

    // initialize ADC but do not start it.
    init_ADC_single_conversion();

    // use adc_burst() to read all of the ADC channels with time delays (non-blocking).
    adc_started_at = milliseconds();
    adc_delay_milsec = cnvrt_milli(ADC_DELAY_MILSEC);
    // in the spin loop place adc_burst(&adc_started_at, &adc_delay_milsec);

    /* Initialize UART to 38.4kbps, it returns a pointer to FILE so redirect of stdin and stdout works*/
    uart1 = uart1_init(38400UL, UART1_RX_REPLACE_CR_WITH_NL);

    /* I2C will be used after some iterations*/
    twim_altPins(); // tell twi0 hardware to use pins PC2, PC3 with MVIO. They go to the R-Pi host
    twi1m_defaultPins(); // tell twi1 hardware to use pins PF2, PF3. They go to the Appliction MCU (e.g., the AVR128DA28)

    // Enable global interrupts to start TIMER0 and UART ISR's
    sei(); 

    // tick count is not milliseconds use cnvrt_milli() to convert time into ticks, thus tickAtomic()/cnvrt_milli(1000) gives seconds
    blink_started_at = tickAtomic();
    blink_delay = cnvrt_milli(BLINK_DELAY);

}

void blink(void)
{
    unsigned long kRuntime = elapsed(&blink_started_at);
    if ( kRuntime > blink_delay)
    {
        ioToggle(MCU_IO_MGR_LED);
        
        // next toggle 
        blink_started_at += blink_delay; 
    }
}

// abort++. 
void abort_safe(void)
{
    // make sure controled devices are safe befor waiting on UART
    ioWrite(MCU_IO_RX_nRE,LOGIC_LEVEL_HIGH); // block RX pair to application RX0 (or its UPDI)
    ioWrite(MCU_IO_RX_DE,LOGIC_LEVEL_LOW); // block host RX from drive RX pair
    ioWrite(MCU_IO_TX_nRE,LOGIC_LEVEL_HIGH); // block TX pair to host
    ioWrite(MCU_IO_TX_DE,LOGIC_LEVEL_LOW); // block application TX0 (or its UPDI) to drive TX pair
    ioWrite(MCU_IO_MGR_SETAPP4_UART,LOGIC_LEVEL_LOW); // disconnect UART
    ioWrite(MCU_IO_MGR_SETAPP4_UPDI,LOGIC_LEVEL_LOW); // disconnect UPDI
    ioWrite(MCU_IO_MGR_LED,LOGIC_LEVEL_LOW);
    // flush the UART befor halt
    uart1_flush();
    twim_off(); // disable TWI0, need to clear the pins
    ioCntl(MCU_IO_MVIO_SCL0, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioCntl(MCU_IO_MVIO_SDA0, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    twi1m_off(); // disable TWI1, need to clear the pins
    ioCntl(MCU_IO_MGR_SCL1, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioCntl(MCU_IO_MGR_SDA1, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    _delay_ms(20); // wait for last byte to send
    uart1_init(0, 0); // disable UART hardware 
    // turn off interrupts and then spin loop a LED toggle 
    cli();
    while(1) 
    {
        _delay_ms(100); 
        ioToggle(MCU_IO_MGR_LED);
    }
}

int main(void) 
{
    setup();

    while(1) {
        if(uart1_available())
        {
            // A standard libc streaming function used for input of one char.
            int input = fgetc(uart1);

            // A standard libc streaming function used for output.
            fprintf(uart1,"%c\r", input); 

            if (input == '$') 
            {
                // Variant of fprintf() that uses a format string which resides in flash memory.
                fprintf_P(uart1,PSTR("{\"abort\":\"'$' found\"}\r\n"));
                abort_safe();
            }

            // press 'a' to stop blinking.
            if(input == 'a')
            {
                got_a = 1;
            }
            else
            {
              got_a = 0;
            }

        }
        if (!got_a)
        {
            blink(); // also ping_i2c1() at the toggle event
        }

        // read all of the ADC channels with time delays (non-blocking)
        adc_burst(&adc_started_at, &adc_delay_milsec);

        // print adc json if stram is available for write
        if (!adc_to_json(uart1, 20000UL)) {
            abort_safe();
        }
    }
    return 0;
}
