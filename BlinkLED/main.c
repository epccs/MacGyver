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
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/io.h>
#include "../lib/uart0_bsd.h"
#include "../lib/io_enum_bsd.h"
#include "../lib/timers_bsd.h"
#include "../lib/twi0_mc.h"

#define BLINK_DELAY 1000UL
unsigned long blink_started_at;
unsigned long blink_delay;

static int got_a;

void setup(void) 
{
    ioCntl(MCU_IO_AIN0, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_AIN0, DIRECTION_OUTPUT);
    ioWrite(MCU_IO_AIN0,LOGIC_LEVEL_HIGH);

    /* Initialize UART to 38.4kbps, it returns a pointer to FILE so redirect of stdin and stdout works*/
    stderr = stdout = stdin = uart0_init(38400UL, UART0_RX_REPLACE_CR_WITH_NL);

    //TCA0_HUNF used for timing, TCA0 split for 6 PWM's, TCB0..TCB2 set for three more PWM's.
    initTimers();

    /* Initialize I2C*/
    //twi1_init(100000UL, TWI1_PINS_PULLUP);

    sei(); // Enable global interrupts to start TIMER0
    
    // tick count is not milliseconds use cnvrt_milli() to convert time into ticks, thus tickAtomic()/cnvrt_milli(1000) gives seconds
    blink_started_at = tickAtomic();
    blink_delay = cnvrt_milli(BLINK_DELAY);

    got_a = 0;
}

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
    if ( kRuntime > blink_delay)
    {
        ioToggle(MCU_IO_AIN0);
        
        // next toggle 
        blink_started_at += blink_delay; 
    }
}

// abort++. 
void abort_safe(void)
{
    // make sure controled devices are safe befor waiting on UART 
    ioDir(MCU_IO_AIN0,DIRECTION_OUTPUT);
    ioWrite(MCU_IO_AIN0,LOGIC_LEVEL_LOW);
    // flush the UART befor halt
    uart0_flush();
    _delay_ms(20); // wait for last byte to send
    uart0_init(0, 0); // disable UART hardware 
    // turn off interrupts and then spin loop a LED toggle 
    cli();
    while(1) 
    {
        _delay_ms(100); 
        ioToggle(MCU_IO_AIN0);
    }
}

int main(void)
{
    setup();

    int abort_yet = 0;

    while (1)
    {
        if(uart0_available())
        {
            // standard C has a libc function getchar() 
            // which gets a byte from stdin.
            // Since I redirected stdin to be from the UART0 this works.
            int input = getchar();

            // standard C has a libc function printf() 
            // which sends a formated string to stdout.
            // stdout was also redirected to UART0, so this also works.
            printf("%c\r", input); 

            if (input == '$') 
            {
                // Variant of printf() that uses a format string that resides in flash memory.
                printf_P(PSTR("{\"abort\":\"egg found\"}\r\n")); 
                abort_safe();
            }

            // press 'a' to stop blinking.
            if(input == 'a') 
            {
                got_a = 1; 
                ++abort_yet; 
            }
            else
            {
              got_a = 0;
            }

            // press 'a' more than five times to hault
            if (abort_yet >= 5) 
            {
                abort_safe();
            }
        }
        if (!got_a)
        {
            blink();
            //i2c_ping();
        }
    }
}

