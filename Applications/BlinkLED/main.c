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
#include <string.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <avr/io.h>
#include "../lib/uart0_bsd.h"
#include "../lib/io_enum_bsd.h"
#include "../lib/timers_bsd.h"
#include "../lib/twi.h"
#include "i2c_monitor.h"

#define BLINK_DELAY 1000UL
unsigned long blink_started_at;
unsigned long blink_delay;

#define TWI_TTL 3000UL
#define TWI_DELAY 5UL
unsigned long twi_started_at_safe; // ISR does not modify
unsigned long twi_ttl;
unsigned long twi_delay;
volatile bool twim_cb_interlock;
uint8_t wrbuf[5];

static int got_a;
FILE *uart0;

//static uint8_t toApp_addr = 40; // app only has one twi port
static uint8_t toMgr_fromApp_addr = 41; // manager-twi1 to application-twi0 
//static uint8_t fromHost_addr = 42; // R-Pi-twi0 to manager-twi0 (mgr has MVIO on the alt twi0 port used)

// finish a twi transaction that was started but is now done
void twimCallback(void) {
    twim_cb_interlock = false;
    twim_callback(NULL); // remove the callback, the ISR checks for NULL and this will cause it to be ignored.
}

// don't block (e.g. _delay_ms(1000) or twim_waitUS() ), ckeck if time has elapsed to toggle
void blink(void)
{
    if (twim_cb_interlock) {
        unsigned long kRuntime = elapsed(&twi_started_at_safe);
        if ( kRuntime > twi_ttl) {
            // timed out
            twim_cb_interlock = false;
            twim_off();
            twim_callback(NULL);
        }
    } else {
        unsigned long kRuntime = elapsed(&blink_started_at);
        if ( kRuntime > blink_delay) {
            ioToggle(MCU_IO_TX2);
            if(ioRead(MCU_IO_TX2)) // write i2c every other toggle
            {
                if (!twim_isBusy()) { // not needed with interlock but whatever
                    twim_callback(twimCallback); // register what to do after transaction has finished
                    twim_on(toMgr_fromApp_addr); //set address, but do not START
                    uint8_t data[] = {'A', 'p', '2', 'M', '\0'};
                    memcpy(wrbuf, data, sizeof(data)); // data will go out of scope (e.g. it lives on the stack)
                    twim_write( wrbuf, sizeof(wrbuf) ); // to address used in twim_on
                    twim_cb_interlock = true;
                    twi_started_at_safe = tickAtomic(); // used for ttl timing, the ISR must not change it
                }
            }

            // next toggle 
            blink_started_at += blink_delay; 
        }
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
    twim_off(); // does not clear the pins
    ioCntl(MCU_IO_SCL0, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioCntl(MCU_IO_SDA0, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
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

    /* Initialize I2C */
    i2c_monitor_init(uart0, uart0_availableForWrite); // this will setup twi pins and slave monitor
    twim_baud( F_CPU, 100000ul ); // setup the master

    twi_delay = cnvrt_milli(TWI_DELAY);
    twi_ttl = cnvrt_milli(TWI_TTL);

    sei(); // Enable global interrupts to start TIMER0

    // tick count is not milliseconds use cnvrt_milli() to convert time into ticks, thus tickAtomic()/cnvrt_milli(1000) gives seconds
    blink_started_at = tickAtomic();
    blink_delay = cnvrt_milli(BLINK_DELAY);

    got_a = 0;
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
            blink(); // also ping_i2c() at the toggle event
        }
        i2c_monitor();
        uint8_t *buf = got_twi0();
        if (buf)
        {
            // no action taken at this time
        }
    }
}

