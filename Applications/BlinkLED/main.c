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
#include <avr/interrupt.h>
#include <avr/io.h>
#include "../lib/uart0_bsd.h"
#include "../lib/io_enum_bsd.h"
#include "../lib/timers_bsd.h"
#include "../lib/twi0_bsd.h"
//#include "../lib/twi0_mc.h"

#define BLINK_DELAY 1000UL
unsigned long blink_started_at;
unsigned long blink_delay;

static int got_a;

FILE *uart0;

// don't block (e.g. _delay_ms(1000) ), ckeck if time has elapsed to toggle 
void blink(void)
{
    unsigned long kRuntime = elapsed(&blink_started_at);
    if ( kRuntime > blink_delay)
    {
        ioToggle(MCU_IO_TX2);
        if(ioRead(MCU_IO_TX2)) // ping i2c every other toggle
        {
            uint8_t mgr_address = 41; //the address I have been useing for the manager (from the application MCU, the host would use 42)
            uint8_t data[] = {'a'};
            uint8_t length = 1;
            twi0_masterBlockingWrite(mgr_address, data, length, TWI0_PROTOCALL_STOP);
            // TWI_MasterWrite(mgr_address,data,length,1);
        }
        
        // next toggle 
        blink_started_at += blink_delay; 
    }
}

// abort++. 
void abort_safe(void)
{
    // make sure controled devices are safe befor waiting on UART 
    ioDir(MCU_IO_TX2,DIRECTION_OUTPUT);
    ioWrite(MCU_IO_TX2,LOGIC_LEVEL_LOW);
    // flush the UART befor halt
    uart0_flush();
    //twi0_init(0, TWI0_PINS_FLOATING); // disable I2C0
    _delay_ms(20); // wait for last byte to send
    uart0_init(0, 0); // disable UART hardware
    // turn off interrupts and then spin loop a LED toggle
    cli();
    while(1)
    {
        _delay_ms(100);
        ioToggle(MCU_IO_TX2);
    }
}

void setup(void)
{
    ioCntl(MCU_IO_TX2, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_TX2, DIRECTION_OUTPUT);
    ioWrite(MCU_IO_TX2,LOGIC_LEVEL_HIGH);

    /* Initialize UART0 to 38.4kbps for streaming, it returns a pointer to a FILE structure */
    uart0 = uart0_init(38400UL, UART0_RX_REPLACE_CR_WITH_NL);

    //TCA0_HUNF used for timing, TCA0 split for 6 PWM's (WO0..WO5).
    initTimers();

    /* Initialize I2C*/
    twi0_init(100000UL, TWI0_PINS_PULLUP); // twi0_bsd
    //TWI_MasterInit(100000UL); // twi0_mc

    sei(); // Enable global interrupts to start TIMER0
    
    // tick count is not milliseconds use cnvrt_milli() to convert time into ticks, thus tickAtomic()/cnvrt_milli(1000) gives seconds
    blink_started_at = tickAtomic();
    blink_delay = cnvrt_milli(BLINK_DELAY);

    got_a = 0;

    uint8_t mgr_address = 41;
    uint8_t data[] = {108};
    uint8_t length = 1;
    twi0_masterBlockingWrite(mgr_address, data, length, TWI0_PROTOCALL_STOP);
    //TWI_MasterWrite(mgr_address,data,length,1);
}

int main(void)
{
    setup();

    while (1)
    {
        if(uart0_available())
        {
            // A standard libc streaming function used for input of one char.
            int input = fgetc(uart0);

            // A standard libc streaming function used for output.
            fprintf(uart0,"%c\r", input); 

            if (input == '$') 
            {
                // Variant of fprintf() that uses a format string which resides in flash memory.
                fprintf_P(uart0,PSTR("{\"abort\":\"'$' found\"}\r\n"));
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
            blink(); // also ping_i2c() at the toggle time
        }
    }
}

