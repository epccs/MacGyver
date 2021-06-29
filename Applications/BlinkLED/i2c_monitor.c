/*
TWI Monitor, output goes to serial debug port
Copyright (c) 2021 Ronald S,. Sutherland

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
#include <stdint.h>
#include <stdio.h>
#include <avr/pgmspace.h>
#include <avr/io.h>
#include "../lib/twi.h"
#include "i2c_monitor.h"

#define BUFF_SIZE 32

static uint8_t toApp_addr = 40; // app only has one twi port
static bool twi0_addr_verified;

static uint8_t BufferA[BUFF_SIZE];
static uint8_t BufferB[BUFF_SIZE];
static uint8_t BufferC[BUFF_SIZE];

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
static uint8_t debug_print_done = 0;

static uint8_t twi0_slave_status_cpy;
#define LAST_OP_A 0
#define LAST_OP_R 1
#define LAST_OP_W 2
static uint8_t twi0_last_op; // last operation e.g., read, write, address
static uint8_t printing;

static bool got_twi0_;
static uint8_t *got_twi0_buf = BufferC;
static uint8_t got_twi0BufferLength;
static uint8_t got_twi0BufferIndex;

static FILE *debug_port;
static streamTx_available available_;

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
            printing = false;
            ret = false;
        }
    }
    return ret;
}

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
            ret = twi0_addr_verified = (twis_lastAddress() == toApp_addr); // test address true to proceed with read or write
            twi0_slave_status_cpy = statusReg;
            if (twi0RxBufferLength) {
                printing = (printOp1BufferIndex >= printOp1BufferLength) && (printOp2BufferIndex >= printOp2BufferLength) && available_();
                print_Op1_buf_if_possible(twi0_last_op, twi0RxBuffer, twi0RxBufferLength, twis_lastAddress()); // print receive buffer as first operation
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
                else { // write+read (echo)
                    print_Op2_buf_if_possible(twi0_last_op, twi0TxBuffer, twi0TxBufferLength, twis_lastAddress());
                    move_buffer(twi0TxBuffer, &twi0TxBufferLength, got_twi0_buf, &got_twi0BufferLength, &got_twi0BufferIndex); // duplicate echo into got_twi0_buf for use in application
                    got_twi0_ = true;
                }
            } else if (twi0RxBufferLength) { // stop after write (read has no data, the slave is ignoring in fact the ACK is not from the slave, the master reads 0xFF and ACKs it, FUBAR)
                printing = (printOp1BufferIndex >= printOp1BufferLength) && (printOp2BufferIndex >= printOp2BufferLength) && available_();
                print_Op1_buf_if_possible(twi0_last_op, twi0RxBuffer, twi0RxBufferLength, twis_lastAddress());
            } else if (twi0_last_op == LAST_OP_A) { // we got a ping
                printing = (printOp1BufferIndex >= printOp1BufferLength) && (printOp2BufferIndex >= printOp2BufferLength) && available_();
                if (printing && twi0_addr_verified) { // just print it now, monitor should do this but...
                    fprintf_P(debug_port,PSTR("{\"ping\":\"0x%X\"}\r\n"),toApp_addr);
                }
            }

            // transaction is done.
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

#if defined(TWI1)
static uint8_t fromApp_addr = 41; // address I have been using for application to connect with the manager
static bool twi1_addr_verified;

static uint8_t BufferD[BUFF_SIZE];
static uint8_t BufferE[BUFF_SIZE];
static uint8_t BufferF[BUFF_SIZE];

static uint8_t *twi1RxBuffer = BufferD;
static uint8_t twi1RxBufferLength;

static uint8_t *twi1TxBuffer = BufferE;
static uint8_t twi1TxBufferLength;
static uint8_t twi1TxBufferIndex;

static uint8_t twi1_slave_status_cpy;
static uint8_t twi1_last_op; // last operation e.g., read, write, address

static uint8_t got_twi1_;
static uint8_t *got_twi1_buf = BufferF;
static uint8_t got_twi1BufferLength;
static uint8_t got_twi1BufferIndex;

bool twi1sCallback(twis_irqstate_t state, uint8_t statusReg) {
    bool ret = true;

    switch( state ) {
        case TWIS_ADDRESSED:
            // at this point, the callback has visibility to all bus addressing, which is interesting.
            ret = twi1_addr_verified = (twi1s_lastAddress() == fromApp_addr); // test address true to proceed with read or write
            twi1_slave_status_cpy = statusReg;
            if (twi1RxBufferLength) {
                printing = (printOp1BufferIndex >= printOp1BufferLength) && (printOp2BufferIndex >= printOp2BufferLength) && available_();
                print_Op1_buf_if_possible(twi1_last_op, twi1RxBuffer, twi1RxBufferLength, twi1s_lastAddress()); // print receive buffer as first operation
                move_buffer(twi1RxBuffer, &twi1RxBufferLength, twi1TxBuffer, &twi1TxBufferLength, &twi1TxBufferIndex); // copy receive buffer into transmit in case next operation is read (so it can echo)
            }
            twi1_last_op = LAST_OP_A;
            break;
        case TWIS_MREAD:
            if (twi1TxBufferIndex < twi1TxBufferLength) {
                twi1s_write( twi1TxBuffer[twi1TxBufferIndex++] );
                ret = true; // more data is in the Tx buffer
            }
            // note if master ignores the NACK and keeps reading 
            // it will get 0xFF since the slave will not pull down on SDA,
            twi1_last_op = LAST_OP_R;
            break;
        case TWIS_MWRITE:
            twi1RxBuffer[twi1RxBufferLength] = twi1s_read();
            ret = (++twi1RxBufferLength < BUFF_SIZE); //true to proceed
            twi1_last_op = LAST_OP_W;
            break;
        case TWIS_STOPPED: 
            if (twi1TxBufferLength) { // stop after
                if (twi1RxBufferLength) { // write+write
                    print_Op2_buf_if_possible(twi1_last_op, twi1RxBuffer, twi1RxBufferLength, twi1s_lastAddress());
                    move_buffer(twi1TxBuffer, &twi1TxBufferLength, got_twi1_buf, &got_twi1BufferLength, &got_twi1BufferIndex); // duplicate echo into got_twi0_buf for use in application
                    got_twi1_ = true;
                }
                else { // write+read
                    print_Op2_buf_if_possible(twi1_last_op, twi1TxBuffer, twi1TxBufferLength, twi1s_lastAddress());
                }
            } else if (twi1RxBufferLength) { // stop after write or read
                printing = (printOp1BufferIndex >= printOp1BufferLength) && (printOp2BufferIndex >= printOp2BufferLength) && available_();
                print_Op1_buf_if_possible(twi1_last_op, twi1RxBuffer, twi1RxBufferLength, twi1s_lastAddress());
            } else if (twi1_last_op == LAST_OP_A) { // we got a ping
                printing = (printOp1BufferIndex >= printOp1BufferLength) && (printOp2BufferIndex >= printOp2BufferLength) && available_();
                if (printing && twi1_addr_verified) { // just print it now,  monitor should do this but...
                    fprintf_P(debug_port,PSTR("{\"ping\":\"0x%X\"}\r\n"),fromApp_addr);
                }
            }

            // transaction is done so reset the buffers
            twi1TxBufferLength = 0;
            twi1RxBufferLength = 0;
            ret = true;
            break;
        case TWIS_ERROR:
            ret = false;
            break;
        }
    return ret;
}
#endif

//==========
// public:
//==========

// init TWI0 (PC2,PC3) and TW1 (PF2,PF3) and get a link to the debug iostream and the test for its Tx buffer availability
void i2c_monitor_init(FILE *debug_uart_to_use, streamTx_available cb) {
    available_ = cb;
    //twim_altPins();             // DB master (and slave) pins are PC2, PC3 with MVIO and go to the R-Pi host
    twim_defaultPins();           // DA master (and slave) pins are PA2, PA3 and go to the DB (PF2, PF3)
    twis_init(toApp_addr, twisCallback );// gencall enabled, so check address in callback
#if defined(TWI1)
    twi1m_defaultPins();             // DB master (and slave) pins are PF2, PF3 and go to the Appliction MCU (e.g., the AVR128DA28)
    twi1s_init(fromApp_addr, twi1sCallback );// gencall enabled, so check address in callback
    got_twi1_ = false;
#endif
    debug_port = debug_uart_to_use;
    got_twi0_ = false;
}

// got TWI, this will give a pointer to buffer and reset flag
uint8_t *got_twi0(void) {
    uint8_t *ret = NULL;
    if (got_twi0_) {
        ret = got_twi0_buf;
        got_twi0_ = false;
    }
    return ret;
}

#if defined(TWI1)
uint8_t * got_twi1(void) {
    uint8_t *ret = NULL;
    if (got_twi1_) {
        ret = got_twi0_buf;
        got_twi1_ = false;
    }
    return ret;
}
#endif

// Monitor the I2C slave address with the debug UART
void i2c_monitor(void)
{
    if (available_()) {
        if ( (debug_print_done == 0) )
        {
            if (printOp1BufferIndex < printOp1BufferLength)
            {
                fprintf_P(debug_port,PSTR("{\"monitor_0x%X\":["),print_slave_addr); // start of JSON for monitor
                debug_print_done = 1;
            }
            else
            {
                return; // slave receive did not happen yet
            }
        }

        else if ( (debug_print_done == 1) ) // twi slave status when transmit_callback is done
        {
            fprintf_P(debug_port,PSTR("{\"status\":\"0x%X\"}"),twi0_slave_status_cpy);
            debug_print_done = 2;
        }

        else if ( (debug_print_done == 2) )
        {
            fprintf_P(debug_port,PSTR(",{\"len\":\"%d\"}"),printOp1BufferLength); 
            debug_print_done = 3;
        }

        else if ( (debug_print_done == 3) ) {
            if (printOp1BufferIndex >= printOp1BufferLength) {
                debug_print_done = 4; // done printing 
            } else {
                if (printOp1rw == LAST_OP_W) {
                    fprintf_P(debug_port,PSTR(",{\"W1\":\"0x%X\"}"),printOp1Buffer[printOp1BufferIndex++]);
                } else if (printOp1rw == LAST_OP_R) {
                    fprintf_P(debug_port,PSTR(",{\"R1\":\"0x%X\"}"),printOp1Buffer[printOp1BufferIndex++]);
                }
            }
        }

        // if the second operation clock stretch is long, this may not print
        else if ( (debug_print_done == 4) ) {
            if (printOp2BufferIndex >= printOp2BufferLength) {
                debug_print_done = 5; // done printing 
            } else {
                if (printOp2rw == LAST_OP_W) {
                    fprintf_P(debug_port,PSTR(",{\"W2\":\"0x%X\"}"),printOp2Buffer[printOp2BufferIndex++]);
                } else if (printOp2rw == LAST_OP_R) {
                    fprintf_P(debug_port,PSTR(",{\"R2\":\"0x%X\"}"),printOp2Buffer[printOp2BufferIndex++]);
                }
            }
        }

        if ( (debug_print_done == 5) )
        {
            fprintf_P(debug_port,PSTR("]}\r\n"));
            debug_print_done = 0; // wait for next slave receive event to fill printBuffer(s)
        }
    }
}