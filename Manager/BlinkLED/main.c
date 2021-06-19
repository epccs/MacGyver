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
#include "../lib/twi.h"

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
    ioDir(MCU_IO_MGR_LED,DIRECTION_OUTPUT);
    ioWrite(MCU_IO_MGR_LED,LOGIC_LEVEL_LOW);
    // flush the UART befor halt
    uart1_flush();
    twim_off(); // need to clear the pins
    ioCntl(MCU_IO_MVIO_SCL0, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioCntl(MCU_IO_MVIO_SDA0, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
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

#define BUFF_SIZE 32

static uint8_t fromHost_addr = 42; // address I have been using for host to connect with the manager on SMBus

static uint8_t BufferA[BUFF_SIZE];
static uint8_t BufferB[BUFF_SIZE];

static uint8_t *twi0RxBuffer = BufferA;
static uint8_t twi0RxBufferLength;

static uint8_t *twi0TxBuffer = BufferB;
static uint8_t twi0TxBufferLength;
static uint8_t twi0TxBufferIndex;

static uint8_t printOp1Buffer[BUFF_SIZE];
static uint8_t printOp1BufferLength;
static uint8_t printOp1BufferIndex;
static uint8_t printOp1rw; // r = 0, w = 1
static uint8_t printOp2Buffer[BUFF_SIZE];
static uint8_t printOp2BufferLength;
static uint8_t printOp2BufferIndex;
static uint8_t printOp2rw; // r = 0, w = 1
static uint8_t print_slave_addr;

static uint8_t twi0_slave_status_cpy;
#define LAST_OP_A 0
#define LAST_OP_R 1
#define LAST_OP_W 2
static uint8_t twi0_last_op; // last operation e.g., read, write, address
static uint8_t printing;

// fill print op1 buffer 
bool print_Op1_buf_if_possible(uint8_t rw, uint8_t buf[], uint8_t bufsize , uint8_t lastAddress) {

    // e.g., printing done and debug uart is free
    bool ret = printing;

    if (ret) {
        printOp1BufferLength = bufsize;
        printOp1BufferIndex = 0;
        for(int i = 0; i < bufsize; ++i)
        {
            printOp1Buffer[i] = buf[i];
        }
        printOp1rw = rw;
        print_slave_addr = lastAddress;
    }
    return ret;
}

// fill print op2 buffer (e.g. write+write/write+read on i2c)
bool print_Op2_buf_if_possible(uint8_t rw, uint8_t buf[], uint8_t bufsize , uint8_t lastAddress) {

    // e.g., printing done and debug uart is free
    bool ret = printing;

    if (ret) {
        printOp2BufferLength = bufsize;
        printOp2BufferIndex = 0;
        for(int i = 0; i < bufsize; ++i)
        {
            printOp2Buffer[i] = buf[i];
        }
        printOp2rw = rw;
        if (print_slave_addr != lastAddress) { // Welp crap, don't print this we have got data for different addresses
            printOp2BufferLength = 0;
            printOp1BufferLength = 0;
        }
    }
    return ret;
}

// move buffer from into to, set s starting index on the to_buffer
bool move_buffer(uint8_t from_buf[], uint8_t *from_bufsize, uint8_t to_buf[], uint8_t *to_bufsize, uint8_t *to_bufindex) {
    bool ret = true;
    for(int i = 0; i < *from_bufsize; ++i) {
        to_buf[i] = from_buf[i];
    }
    *to_bufsize = *from_bufsize;
    *from_bufsize = 0;
    *to_bufindex = 0; // used for read to index
    return ret;
}

bool twisCallback(twis_irqstate_t state, uint8_t statusReg) {
    bool ret = true;

    switch( state ) {
        case TWIS_ADDRESSED:
            // at this point, the callback has visibility to all bus addressing, which is interesting.
            ret = (twis_lastAddress() == fromHost_addr); // test address true to proceed with read or write
            twi0_slave_status_cpy = statusReg;
            if (twi0RxBufferLength) {
                printing = (printOp1BufferIndex >= printOp1BufferLength) && (printOp2BufferIndex >= printOp2BufferLength) && uart1_availableForWrite();
                print_Op1_buf_if_possible(twi0_last_op, twi0RxBuffer, twi0RxBufferLength, twis_lastAddress()); // print reciece buffer as first operation
                move_buffer(twi0RxBuffer, &twi0RxBufferLength, twi0TxBuffer, &twi0TxBufferLength, &twi0TxBufferIndex); // copy receive buffer into transmit in case next operation is read (so it can echo)
            }
            twi0_last_op = LAST_OP_A;
            break;
        case TWIS_MREAD:
            if (twi0TxBufferIndex < twi0TxBufferLength) {
                twis_write( twi0TxBuffer[twi0TxBufferIndex++] );
                ret = true; // more data is in the Tx buffer
            }
            // note if master ignores the NACK and keeps reading 
            // it will get 0xFF since the slave will not pull down on SDA,
            twi0_last_op = LAST_OP_R;
            break;
        case TWIS_MWRITE:
            twi0RxBuffer[twi0RxBufferLength] = twis_read();
            ret = (++twi0RxBufferLength < BUFF_SIZE); //true to proceed
            twi0_last_op = LAST_OP_W;
            break;
        case TWIS_STOPPED: 
            if (twi0TxBufferLength) { // stop after
                if (twi0RxBufferLength) { // write+write
                    print_Op2_buf_if_possible(twi0_last_op, twi0RxBuffer, twi0RxBufferLength, twis_lastAddress());
                }
                else { // write+read
                    print_Op2_buf_if_possible(twi0_last_op, twi0TxBuffer, twi0TxBufferLength, twis_lastAddress());
                }
            } else if (twi0RxBufferLength) { // stop after write or read
                printing = (printOp1BufferIndex >= printOp1BufferLength) && (printOp2BufferIndex >= printOp2BufferLength) && uart1_availableForWrite();
                print_Op1_buf_if_possible(twi0_last_op, twi0RxBuffer, twi0RxBufferLength, twis_lastAddress());
            } else if (twi0_last_op == LAST_OP_A) { // we got a ping
                printing = (printOp1BufferIndex >= printOp1BufferLength) && (printOp2BufferIndex >= printOp2BufferLength) && uart1_availableForWrite();
                if (printing) { // just print it now,  monitor should do this but...
                    fprintf_P(uart1,PSTR("{\"ping\":\"0x%X\"}\r\n"),fromHost_addr);
                }
            }

            // transaction is done so reset the buffers
            twi0TxBufferLength = 0;
            twi0RxBufferLength = 0;
            ret = true;
            break;
        case TWIS_ERROR:
            ret = false;
            break;
        }
    return ret;
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

    /* Initialize I2C */
    twim_altPins();             // master (and slave) pins are PC2, PC3 with MVIO and go to the R-Pi host
    twis_init(fromHost_addr, twisCallback );// gencall enabled, so check address in callback

    sei(); // Enable global interrupts to start TIMER0
    
    // tick count is not milliseconds use cnvrt_milli() to convert time into ticks, thus tickAtomic()/cnvrt_milli(1000) gives seconds
    blink_started_at = tickAtomic();
    blink_delay = cnvrt_milli(BLINK_DELAY);

    got_a = 0;
}

uint8_t debug_print_done = 0;

// Monitor the I2C slave address with the debug UART
void i2c_monitor(void)
{
    if ( (debug_print_done == 0) )
    {
        if (printOp1BufferIndex < printOp1BufferLength)
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
        fprintf_P(uart1,PSTR("{\"status\":\"0x%X\"}"),twi0_slave_status_cpy);
        debug_print_done = 2;
    }

    else if ( (debug_print_done == 2) )
    {
        fprintf_P(uart1,PSTR(",{\"len\":\"%d\"}"),printOp1BufferLength); 
        debug_print_done = 3;
    }

    else if ( (debug_print_done == 3) ) {
        if (printOp1BufferIndex >= printOp1BufferLength) {
            debug_print_done = 4; // done printing 
        } else {
            if (printOp1rw == LAST_OP_W) {
                fprintf_P(uart1,PSTR(",{\"W1\":\"0x%X\"}"),printOp1Buffer[printOp1BufferIndex++]);
            } else if (printOp1rw == LAST_OP_R) {
                fprintf_P(uart1,PSTR(",{\"R1\":\"0x%X\"}"),printOp1Buffer[printOp1BufferIndex++]);
            }
        }
    }

    // if the second operation clock stretch is long, this may not print
    else if ( (debug_print_done == 4) ) {
        if (printOp2BufferIndex >= printOp2BufferLength) {
            debug_print_done = 5; // done printing 
        } else {
            if (printOp2rw == LAST_OP_W) {
                fprintf_P(uart1,PSTR(",{\"W2\":\"0x%X\"}"),printOp2Buffer[printOp2BufferIndex++]);
            } else if (printOp2rw == LAST_OP_R) {
                fprintf_P(uart1,PSTR(",{\"R2\":\"0x%X\"}"),printOp2Buffer[printOp2BufferIndex++]);
            }
        }
    }

    if ( (debug_print_done == 5) )
    {
        fprintf_P(uart1,PSTR("]}\r\n"));
        debug_print_done = 0; // wait for next slave receive event to fill printBuffer(s)
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
            i2c_monitor();
        }
        if (!got_a)
        {
            blink(); // also ping_i2c() at the toggle time
        }
    }
}

