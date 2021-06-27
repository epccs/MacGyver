/* Application Upload
Copyright (C) 2021 Ronald Sutherland

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
#include "../lib/uart1_bsd.h"
#include "../lib/io_enum_bsd.h"
#include "../lib/timers_bsd.h"
#include "../lib/twi.h"
#include "i2c_monitor.h"

#define BLINK_DELAY 1000UL
unsigned long blink_started_at;
unsigned long blink_delay;

static int got_a;

FILE *uart1;

// don't block (e.g. _delay_ms(1000) ), ckeck if time has elapsed to toggle 
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

void setup(void)
{
    ioCntl(MCU_IO_MGR_LED, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_MGR_LED, DIRECTION_OUTPUT);
    ioWrite(MCU_IO_MGR_LED,LOGIC_LEVEL_HIGH);

    /* Manager controls if multi-drop RX and TX transceivers */
    ioCntl(MCU_IO_RX_nRE, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_RX_nRE, DIRECTION_OUTPUT);
    ioWrite(MCU_IO_RX_nRE,LOGIC_LEVEL_LOW); // send RX pair to application RX0 (or its UPDI)
    ioCntl(MCU_IO_RX_DE, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_RX_DE, DIRECTION_OUTPUT);
    ioWrite(MCU_IO_RX_DE,LOGIC_LEVEL_HIGH); // allow host RX to drive RX pair low (if host RX is high drive is disabled)
    ioCntl(MCU_IO_TX_nRE, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_TX_nRE, DIRECTION_OUTPUT);
    ioWrite(MCU_IO_TX_nRE,LOGIC_LEVEL_LOW); // send TX pair to host
    ioCntl(MCU_IO_TX_DE, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_TX_DE, DIRECTION_OUTPUT);
    ioWrite(MCU_IO_TX_DE,LOGIC_LEVEL_HIGH); // allow application TX0 (or its UPDI) to drive TX pair low (if TX0 is high drive is disabled)

    /* Manager controls out of band transceiver, which is not used for this program*/
    ioCntl(MCU_IO_OOB_nRE, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_OOB_nRE, DIRECTION_OUTPUT);
    ioWrite(MCU_IO_OOB_nRE,LOGIC_LEVEL_HIGH); // manager does not receive from OOB pair
    ioCntl(MCU_IO_OOB_DE, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_OOB_DE, DIRECTION_OUTPUT);
    ioWrite(MCU_IO_OOB_DE,LOGIC_LEVEL_LOW); // manager does not send to OOB pair

    /* Manager controls if multi-drop is connected to UART or UPDI */
    ioCntl(MCU_IO_MGR_SETAPP4_UART, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_MGR_SETAPP4_UART, DIRECTION_OUTPUT);
    ioWrite(MCU_IO_MGR_SETAPP4_UART,LOGIC_LEVEL_HIGH); // connect to UART
    ioCntl(MCU_IO_MGR_SETAPP4_UPDI, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_MGR_SETAPP4_UPDI, DIRECTION_OUTPUT);
    ioWrite(MCU_IO_MGR_SETAPP4_UPDI,LOGIC_LEVEL_LOW); // disconnect UPDI

    /* Initialize UART1 to 38.4kbps for streaming, it returns a pointer to a FILE structure*/
    uart1 = uart1_init(38400UL, UART1_RX_REPLACE_CR_WITH_NL);

    //TCA0_HUNF used for timing, TCA0 split for 6 PWM's.
    initTimers();

    /* Initialize I2C monitor (includes the twis callback's) */
    i2c_monitor_init(uart1, uart1_availableForWrite);

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
            blink(); // also ping_i2c() at the toggle time
        }
        i2c_monitor();
        uint8_t *buf = got_twi0();
        if (buf) // only if write+read is done can the host change UPDI mode for Application programing
        {
            if (buf[0] == 7) // if command byte is 7 on SMBus from host
            {
                // UPDI mode, application uploaded over multi-drop serial
                ioWrite(MCU_IO_MGR_SETAPP4_UART,LOGIC_LEVEL_LOW); // disconnect to UART
                ioWrite(MCU_IO_MGR_SETAPP4_UPDI,LOGIC_LEVEL_HIGH); // connect UPDI
                blink_delay = cnvrt_milli(BLINK_DELAY/4);
            }
            else
            {
                // UART mode, application serial connected to multi-drop serial
                ioWrite(MCU_IO_MGR_SETAPP4_UART,LOGIC_LEVEL_HIGH); // connect to UART
                ioWrite(MCU_IO_MGR_SETAPP4_UPDI,LOGIC_LEVEL_LOW); // disconnect UPDI
                blink_delay = cnvrt_milli(BLINK_DELAY);
            }
        }
        buf = got_twi1();
        if (buf) {
            // no action taken at this time
        }
    }
}

