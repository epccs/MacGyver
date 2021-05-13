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
#include "../lib/uart1_bsd.h"
#include "../lib/io_enum_bsd.h"
#include "../lib/timers_bsd.h"
#include "../lib/twi0_bsd.h"

#define BLINK_DELAY 1000UL
unsigned long blink_started_at;
unsigned long blink_delay;

static int got_a;

FILE *uart1;

// cycle the twi state machine
/* This is a place holder for now
   the master mode is not used with the SMBus connection between R-Pi and manager
*/
void i2c_ping(void)
{ 
    // ping I2C for a manager 
    uint8_t mgr_address = 41; //the address I have been useing for the manager from the application MCU
    uint8_t data[] = {0};
    uint8_t length = 1;
    for (uint8_t i=0; i<5; i++) // try a few times.
    {
        uint8_t twi_errorCode = twi0_masterBlockingWrite(mgr_address, data, length, TWI0_PROTOCALL_STOP); 
        if (twi_errorCode == 0) break; // ping was error free
    }
    return; 
}

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
    ioDir(MCU_IO_MGR_LED,DIRECTION_OUTPUT);
    ioWrite(MCU_IO_MGR_LED,LOGIC_LEVEL_LOW);
    // flush the UART befor halt
    uart1_flush();
    twi0_init(0, TWI0_PINS_FLOATING); // disable I2C0
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


static uint8_t slave_addr = 42; // address I have been using for host to connect with the manager on SMBus

static uint8_t localBuffer[TWI0_BUFFER_LENGTH];
static uint8_t localBufferLength;

static uint8_t printBuffer[TWI0_BUFFER_LENGTH];
static uint8_t printBufferLength;
static uint8_t printBufferIndex;

static uint8_t twi0_slave_status_cpy;

// echo what was received
void twi0_transmit_callback(void)
{
    twi0_slave_status_cpy = TWI0.SSTATUS;
    /*uint8_t return_code = */ twi0_fillSlaveTxBuffer(localBuffer, localBufferLength);
    
    /* todo: add status_byt to main so I can use it to blink the LED fast or for abort
    if (return_code != 0)
        status_byt &= (1<<DTR_I2C_TRANSMIT_FAIL);
    */
    return;
}

// Place the received data in local buffer so it can echo back.
// If monitor is running, printing done, and UART is available 
// fill the print buffer and reset the index for printing
void twi0_receive_callback(uint8_t *data, uint8_t length)
{
    localBufferLength = length;
    for(int i = 0; i < length; ++i)
    {
        localBuffer[i] = data[i];
    }
    if (slave_addr && (printBufferLength == printBufferIndex) && uart1_availableForWrite())
    {
        printBufferLength = length;
        printBufferIndex = 0;
        for(int i = 0; i < length; ++i)
        {
            printBuffer[i] = data[i];
        }
    }
    return;
}

void setup(void)
{
    ioCntl(MCU_IO_MGR_LED, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_MGR_LED, DIRECTION_OUTPUT);
    ioWrite(MCU_IO_MGR_LED,LOGIC_LEVEL_HIGH);

    /* Initialize UART1 to 38.4kbps for streaming, it returns a pointer to a FILE structure */
    uart1 = uart1_init(38400UL, UART1_RX_REPLACE_CR_WITH_NL);

    //TCA0_HUNF used for timing, TCA0 split for 6 PWM's.
    initTimers();

    /* Initialize I2C master*/
    twi0_init(100000UL, TWI0_PINS_PULLUP); // twi0_bsd
    //TWI_MasterInit(100000UL); // twi0_mc

    /* Initialize I2C slave*/
    twi0_registerSlaveRxCallback(twi0_receive_callback);
    twi0_registerSlaveTxCallback(twi0_transmit_callback);
    twi0_slaveAddress(slave_addr); // ISR is enabled so register callback first

    sei(); // Enable global interrupts to start TIMER0
    
    // tick count is not milliseconds use cnvrt_milli() to convert time into ticks, thus tickAtomic()/cnvrt_milli(1000) gives seconds
    blink_started_at = tickAtomic();
    blink_delay = cnvrt_milli(BLINK_DELAY);

    got_a = 0;
}

uint8_t debug_print_done = 0;

// Monitor the I2C slave address with the debug UART
void I2c0_monitor(void)
{
    if ( (debug_print_done == 0) )
    {
        if (printBufferIndex < printBufferLength)
        {
            fprintf_P(uart1,PSTR("{\"monitor_0x%X\":["),slave_addr); // start of JSON for monitor
            debug_print_done = 1;
        }
        else
        {
            return; // slave receive did not happen yet
        }
    }

    else if ( (debug_print_done == 1) ) // twi slave status when transmit_callback is done
    {
        fprintf_P(uart1,PSTR("{\"status\":\"%d\"}"),twi0_slave_status_cpy);
        debug_print_done = 2;
    }

    else if ( (debug_print_done == 2) )
    {
        fprintf_P(uart1,PSTR(",{\"len\":\"%d\"}"),printBufferLength - 1); 
        debug_print_done = 3;
    }

    else if ( (debug_print_done == 3) )
    {
        printBufferIndex += 1;
        if (printBufferIndex >= printBufferLength) 
        {
            debug_print_done = 4; // done printing 
        }
        else
        {
            fprintf_P(uart1,PSTR(",{\"dat\":\"0x%X\"}"),printBuffer[printBufferIndex-1]);
        }
    }
    
    if ( (debug_print_done == 4) )
    {
        fprintf_P(uart1,PSTR("]}\r\n"));
        debug_print_done = 0; // wait for next slave receive event to fill printBuffer
    }
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
        if(uart1_availableForWrite())
        {
            I2c0_monitor();
        }
        if (!got_a)
        {
            blink(); // also ping_i2c() at the toggle time
        }
    }
}

