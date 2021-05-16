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
#include "../lib/twi0_bsd.h"
#include "../lib/twi1_bsd.h"

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
    twi0_init(0, TWI0_PINS_FLOATING); // disable I2C0
    twi1_init(0, TWI1_PINS_FLOATING); // disable I2C0
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

static uint8_t twi0_slave_addr = 42; // address I have been using for host to connect with the manager on SMBus

static uint8_t twi0_localBuffer[TWI0_BUFFER_LENGTH];
static uint8_t twi0_localBufferLength;
static uint8_t twi0_cpy_of_txstatus;

static uint8_t printBuffer[TWI0_BUFFER_LENGTH];
static uint8_t printBufferLength;
static uint8_t printBufferIndex;
static uint8_t print_slave_addr;
static uint8_t print_slave_status;

static uint8_t got_twi0;

// echo what was received
void twi0_transmit_callback(void)
{
    twi0_cpy_of_txstatus = TWI0.SSTATUS;
    twi0_fillSlaveTxBuffer(twi0_localBuffer, twi0_localBufferLength);
    return;
}

// Place the received data in local buffer so it can echo back.
// If monitor is running, printing done, and UART is available 
// fill the print buffer and reset the index for printing
void twi0_receive_callback(uint8_t *data, uint8_t length)
{
    twi0_localBufferLength = length;
    for(int i = 0; i < length; ++i)
    {
        twi0_localBuffer[i] = data[i];
    }
    got_twi0 = 1;
    if ((printBufferLength == printBufferIndex) && uart1_availableForWrite())
    {
        printBufferLength = length;
        printBufferIndex = 0;
        print_slave_addr = twi0_slave_addr;
        print_slave_status = twi0_cpy_of_txstatus;
        for(int i = 0; i < length; ++i)
        {
            printBuffer[i] = data[i];
        }
    }
    return;
}

static uint8_t twi1_slave_addr = 41; // address I have been using for application to connect with the manager

static uint8_t twi1_localBuffer[TWI1_BUFFER_LENGTH];
static uint8_t twi1_localBufferLength;
static uint8_t twi1_cpy_of_txstatus;

static uint8_t got_twi1;

// echo what was received
void twi1_transmit_callback(void)
{
    twi1_cpy_of_txstatus =  TWI1.SSTATUS;
    twi1_fillSlaveTxBuffer(twi1_localBuffer, twi1_localBufferLength);
    return;
}

// Place the received data in local buffer so it can echo back.
// If monitor is running, printing done, and UART is available 
// fill the print buffer and reset the index for printing
void twi1_receive_callback(uint8_t *data, uint8_t length)
{
    twi1_localBufferLength = length;
    for(int i = 0; i < length; ++i)
    {
        twi1_localBuffer[i] = data[i];
    }
    got_twi1 = 1;
    if ((printBufferLength == printBufferIndex) && uart1_availableForWrite())
    {
        printBufferLength = length;
        printBufferIndex = 0;
        print_slave_addr = twi1_slave_addr;
        print_slave_status = twi1_cpy_of_txstatus;
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

    /* Initialize I2C*/
    twi0_init(100000UL, TWI0_PINS_PULLUP); // twi0_bsd
    twi1_init(100000UL, TWI0_PINS_PULLUP); // twi1_bsd

    /* Initialize I2C client*/
    twi0_registerSlaveRxCallback(twi0_receive_callback);
    twi0_registerSlaveTxCallback(twi0_transmit_callback);
    twi0_slaveAddress(twi0_slave_addr); // ISR is enabled so register callback first
    //twi1_registerSlaveRxCallback(twi1_receive_callback);
    //twi1_registerSlaveTxCallback(twi1_transmit_callback);
    //twi1_slaveAddress(twi1_slave_addr);

    sei(); // Enable global interrupts to start TIMER0
    
    // tick count is not milliseconds use cnvrt_milli() to convert time into ticks, thus tickAtomic()/cnvrt_milli(1000) gives seconds
    blink_started_at = tickAtomic();
    blink_delay = cnvrt_milli(BLINK_DELAY);

    got_a = 0;
    got_twi0 = 0;
    got_twi1 = 0;
}

uint8_t debug_print_done = 0;

// Monitor for the I2C clients, output to the debug UART
void i2c_monitor(void)
{
    if ( (debug_print_done == 0) )
    {
        if (printBufferIndex < printBufferLength)
        {
            fprintf_P(uart1,PSTR("{\"monitor_0x%X\":["),print_slave_addr); // start of JSON for monitor
            debug_print_done = 1;
        }
        else
        {
            return; // slave receive did not happen yet
        }
    }

    else if ( (debug_print_done == 1) ) // twi slave status when transmit_callback is done
    {
        fprintf_P(uart1,PSTR("{\"status\":\"0x%X\"}"),print_slave_status);
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
        if (!got_a)
        {
            blink(); // also ping_i2c() at the toggle time
        }
        if(uart1_availableForWrite())
        {
            i2c_monitor();
            if (got_twi1) got_twi1 = 0;
            if (got_twi0)
            {
                if (twi0_localBuffer[0] == 7) // if command byte is 7 on SMBus from host
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
                got_twi0=0;
            }
        }
    }
}

