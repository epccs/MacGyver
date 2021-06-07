/*
AVR Interrupt-Driven Asynchronous I2C C library 
Copyright (C) 2020 Ronald Sutherland

Parts of this were derived from Microchip software which requires that 
derivatives be exclusively used with Microchip products.

Otherwise permission to use, copy, modify, and/or distribute this software for any 
purpose with or without fee is hereby granted.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES 
WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE 
FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY 
DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, 
WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, 
ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

https://en.wikipedia.org/wiki/BSD_licenses#0-clause_license_(%22Zero_Clause_BSD%22)

An AVR128D[AB] has two ISR driven state machines, one for the master and another for the slave.

done: removed interleaving buffer twi0_slaveRxBufferA twi0_slaveRxBufferB use twi0_slaveRxBuffer as the buffer

*/

#include <stdbool.h>
#include <stddef.h>
#include <avr/interrupt.h>
#include <util/twi.h>
// uart0_bsd is for debuging
#include "uart0_bsd.h"
#include "../lib/timers_bsd.h"
#include "io_enum_bsd.h"
#include "twi0_bsd.h"

/*! For adding R/_W bit to address */
#define ADD_READ_BIT(address)    (address | 0x01)
#define ADD_WRITE_BIT(address)  (address & ~0x01)

/*! TWI Modes */
typedef enum TWI_MODE_enum {
    TWI_MODE_UNKNOWN,
    TWI_MODE_MASTER,
    TWI_MODE_DUAL,
    TWI_MODE_MASTER_TRANSMIT,
    TWI_MODE_MASTER_RECEIVE
    //TWI_MODE_SLAVE_TRANSMIT,
    //TWI_MODE_SLAVE_RECEIVE
} TWI_MODE_t;

static volatile TWI_MODE_t twi_mode;

// TWI modes for master module
typedef enum TWIM_MODE_enum
{
    TWIM_MODE_UNKNOWN,
    TWIM_MODE_ENABLE, // master ready to start Rx/Tx
    TWIM_MODE_TRANSMIT, // master is doing Tx
    TWIM_MODE_RECEIVE // master is doing Rx
} TWIM_MODE_t;

static volatile TWIM_MODE_t twim_mode;

// TWI modes for slave module
typedef enum TWIS_MODE_enum 
{
    TWIS_MODE_UNKNOWN,
    TWIS_MODE_ENABLE, // slave ready for Rx/Tx event
    TWIS_MODE_TRANSMIT, // slave is doing Tx event
    TWIS_MODE_RECEIVE // slave is doing Rx event
} TWIS_MODE_t;

static volatile TWIS_MODE_t twis_mode;


typedef enum TWI_ACK_enum {
    TWI_ACK,
    TWI_NACK
} TWI_ACK_t;

static volatile TWI0_PROTOCALL_t twi0_protocall; 

typedef enum TWI_ERROR_enum {
    TWI_ERROR_ILLEGAL = TW_BUS_ERROR, // illegal start or stop condition
    TWI_ERROR_MT_SLAVE_ADDR_NACK = TW_MT_SLA_NACK, // Master Transmiter SLA+W transmitted, NACK received 
    TWI_ERROR_MT_DATA_NACK = TW_MT_DATA_NACK, // Master Transmiter data transmitted, NACK received
    TWI_ERROR_MT_DATA_MISSING = 0x32, // NULL pointer passed rather than the data
    TWI_ERROR_ARBITRATION_LOST = TW_MT_ARB_LOST, // Master Transmiter arbitration lost in SLA+W or data
    TWI_ERROR_MS_SLAVE_ADDR_NACK = TW_MR_SLA_NACK, // Master Receiver SLA+W transmitted, NACK received 
    TWI_ERROR_MS_DATA_NACK = TW_MR_DATA_NACK, // Master Receiver data received, NACK returned ()
    // TW_MR_ARB_LOST is done with TW_MT_ARB_LOST
    TWI_ERROR_NONE = 0xFF // No errors
} TWI_ERROR_t;

static volatile TWI_ERROR_t twi0_error;

static uint8_t twi0_masterBuffer[TWI0_BUFFER_LENGTH];
static volatile uint8_t twi0_masterBufferIndex;
static volatile uint8_t twi0_masterBufferLength;
static volatile uint8_t  master_slaveAddress;
static volatile uint8_t *master_writeData;
static volatile uint8_t *master_readData;
static volatile uint8_t  master_bytesToWrite;
static volatile uint8_t  master_bytesToRead;
static volatile uint8_t  master_bytesWritten;
static volatile uint8_t  master_bytesRead;
static volatile uint8_t  master_sendStop;
static volatile uint8_t  master_trans_status;
static volatile uint8_t master_result;

static uint8_t twi0_slaveTxBuffer[TWI0_BUFFER_LENGTH];
static uint8_t twi0_slaveRxBuffer[TWI0_BUFFER_LENGTH];
static volatile uint8_t  slave_bytesToWrite;
static volatile uint8_t  slave_bytesWritten;
static volatile uint8_t  slave_bytesRead;
static volatile uint8_t  slave_trans_status; // this is does noting and can be removed
static volatile uint8_t  slave_result;  // value are from TWI0S_RESULT_enum
static volatile uint8_t  run_user_receive_callback_after_STOP_or_REPSTART; // preventing clock streatching

// used to initalize the slave Transmit function in case it is not used.
void twi0_transmit_default(void)
{
    // In a real callback, the data to send needs to be copied from a local buffer.
    // uint8_t return_code = twi0_fillSlaveTxBuffer(localBuffer, localBufferLength);

    // this default puts a NUL byte in the buffer and bypasses the provided function
    slave_bytesToWrite = 1;
    twi0_slaveTxBuffer[0] = 0x00;
    return;
}

// used to initalize the slave Receive function in case is not used.
void twi0_receive_default(uint8_t *data, uint8_t length)
{
    // In a real callback, the data to send needs to be copied to a local buffer.
    // 
    // for(int i = 0; i < length; ++i)
    // {
    //     localBuffer[i] = data[i];
    // }
    //
    // the receive event happens once after the I2C stop or repeated-start
    //
    // repeated-start is usd for atomic bus operation e.g. prevents others from using bus
    return;
}

typedef void (*PointerToTransmit)(void);
typedef void (*PointerToReceive)(uint8_t*, uint8_t);
static PointerToTransmit twi0_onSlaveTx = twi0_transmit_default; // user must call twi0_fillSlaveTxBuffer(bytes, length) in callback
static PointerToReceive twi0_onSlaveRx = twi0_receive_default;

// Set the BAUD bit field in the TWIn.MBAUD
void twi0_SetBaud(uint32_t f_scl)
{
    uint16_t t_rise; // is determined by the bus impedance and pull-up, this is only a ball-park.
    
    if(f_scl < 200000) {
        f_scl = 100000; // 100kHz
        t_rise = 1000; // 1000ns, must be bellow 4700ns for equation 2 in DS
    } else if (f_scl < 800000) {
        f_scl = 400000; // 400kHz
        t_rise = 300; // 300ns, must be bellow 1300ns for equation 2 in DS
    } else if (f_scl < 1200000) {
        f_scl = 1000000; // 1MHz
        t_rise = 120; // 120ns, must be bellow 500ns for equation 2 in DS
    } else {
        f_scl = 100000;
        t_rise = 1000; 
    }
    
    // from equation 2 DS40002183B-page 401
    uint32_t baud = (F_CPU/(2*f_scl)) - ( (5 + (((F_CPU/1000)/1000)*t_rise)/1000 )/2 );
    TWI0.MBAUD = (uint8_t)baud;
}

void twi0_MasterTransactionFinished(TWI0M_RESULT_t result)
{
    master_result = result;
    master_trans_status = TWI0M_STATUS_READY;
    twim_mode = TWIM_MODE_ENABLE;
}

// TWI0 Master event
ISR(TWI0_TWIM_vect) {
    uint8_t currentStatus = TWI0.MSTATUS;

    // arbitration lost or bus error.
    if ((currentStatus & TWI_ARBLOST_bm) || (currentStatus & TWI_BUSERR_bm)) {
        uint8_t currentStatus = TWI0.MSTATUS;

        if (currentStatus & TWI_BUSERR_bm) { // bus error
            master_result = TWI0M_RESULT_BUS_ERROR;
        } else { // arbitration lost
            master_result = TWI0M_RESULT_ARBITRATION_LOST;
        }

        // Clear all flags, abort operation
        TWI0.MSTATUS = currentStatus;

        // Wait for a new operation
        twim_mode = TWIM_MODE_ENABLE;
        master_trans_status = TWI0M_STATUS_READY;
    }

    // master write interrupt
    else if (currentStatus & TWI_WIF_bm) 
    {
        uint8_t local_bytesToWrite  = master_bytesToWrite;
        uint8_t local_bytesToRead   = master_bytesToRead;

        // If NOT acknowledged (NACK) by slave cancel the transaction.
        if (TWI0.MSTATUS & TWI_RXACK_bm) {
            if(master_sendStop) {
                TWI0.MCTRLB = TWI_MCMD_STOP_gc;
            } else {
                TWI0.MCTRLB = TWI_MCMD_REPSTART_gc;
            }
            
             // if nil bytes Written must be an address NACK
            if ( (local_bytesToRead > 0) || (local_bytesToWrite > 0) ) {
                twi0_error = TWI_ERROR_MT_SLAVE_ADDR_NACK; 
            } else { // otherwise data NACK
                twi0_error = TWI_ERROR_MT_DATA_NACK;
            }
            twi0_MasterTransactionFinished(TWI0M_RESULT_NACK_RECEIVED);
        }

        //  acknowledged (ACK) by slave, If more bytes to write then send next.
        else if (master_bytesWritten < local_bytesToWrite) {
            uint8_t data = master_writeData[master_bytesWritten];
            TWI0.MDATA = data;
            master_bytesWritten++;
        }

        // Acknowledged (ACK) by slave without more data to send. Maybe bytes to read?
        // i2c read frame looks like: START condition + (Address + 'R/_W = 1')
        else if (master_bytesRead < local_bytesToRead) {
            twi_mode = TWI_MODE_MASTER_RECEIVE;
            twim_mode = TWIM_MODE_RECEIVE;
            uint8_t readAddress = ADD_READ_BIT(master_slaveAddress);
            TWI0.MADDR = readAddress;
        }

        // If finished (or master_writeData is NULL), transaction done.
        else {
            if(master_sendStop) { // The receiving unit will ACK or NACK what is writen
                TWI0.MCTRLB = TWI_MCMD_STOP_gc;
            } else {
                TWI0.MCTRLB = TWI_MCMD_REPSTART_gc;
            }
            twi0_MasterTransactionFinished(TWI0M_RESULT_OK);
        }
    }

    //master read 
    else if (currentStatus & TWI_RIF_bm) {
        if (master_bytesRead < master_bytesToRead) {
            uint8_t data = TWI0.MDATA;
            master_readData[master_bytesRead] = data;
            master_bytesRead++;
        } else { // to many bytes, transaction done, issue NACK (1 to ACKACT bit) and STOP or REPSTART condition.
            if(master_sendStop) {
                TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_STOP_gc;
            } else {
                TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_REPSTART_gc;
            }

            twi0_MasterTransactionFinished(TWI0M_RESULT_BUFFER_OVERFLOW);
            master_bytesToRead = 0;
            return;
        }

        // Local variable used in if test to avoid compiler warning.
        uint8_t bytesToRead = master_bytesToRead;

        //  If more bytes to read, issue ACK and start a byte read.
        if (master_bytesRead < bytesToRead) {
            TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc; // issue ACK to start next byte
        } else { // transaction done, issue NACK (1 to ACKACT bit) and STOP or REPSTART condition.
            if(master_sendStop) {
                TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_STOP_gc;
            } else {
                TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_REPSTART_gc;
            }
            twi0_MasterTransactionFinished(TWI0M_RESULT_OK);
        }
    }

    // unexpected 
    else
    {
        twi0_error = TWI_ERROR_ILLEGAL;
        twi0_MasterTransactionFinished(TWI0M_RESULT_FAIL);
    }
}

void TWI0_SlaveAddressMatchHandler()
{
    slave_trans_status = TWI0S_STATUS_BUSY;
    slave_result = TWI0S_RESULT_UNKNOWN;
    
    // ACK, data should be on the way
    TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
    
    if(TWI0.SSTATUS & TWI_DIR_bm) // Slave Writes to Master
    { 
        slave_bytesWritten = 0;
        /* Call user function  */
        twi0_onSlaveTx(); // user must call twi0_fillSlaveTxBuffer(bytes, length) in callback
        twis_mode = TWIS_MODE_TRANSMIT;
    } 
    else // Master Writes to Slave
    {
        slave_bytesRead = 0;
        run_user_receive_callback_after_STOP_or_REPSTART = 1;
        twis_mode = TWIS_MODE_RECEIVE;
    }
}

void TWI0_SlaveTransactionFinished(TWI0S_RESULT_t result)
{
    TWI0.SCTRLA |= (TWI_APIEN_bm | TWI_PIEN_bm);
    twis_mode = TWIS_MODE_ENABLE;
    slave_result = result;
    slave_trans_status = TWI0S_STATUS_READY;
}

void TWI0_SlaveStopHandler()
{
    // Clear Address/Stop Interrupt Flag, don't ACK or NACK
    TWI0.SSTATUS = TWI_APIF_bm;
    TWI0_SlaveTransactionFinished(TWI0S_RESULT_OK);
}

// TWI0 Slave event
ISR(TWI0_TWIS_vect)
{
    uint8_t currentStatus = TWI0.SSTATUS;
    
    //bus error
    if(currentStatus & TWI_BUSERR_bm) {
        slave_bytesRead = 0;
        slave_bytesWritten = 0;
        slave_bytesToWrite = 0;
        TWI0_SlaveTransactionFinished(TWI0S_RESULT_BUS_ERROR);
    }
    
    // Address or Stop
    else if(currentStatus & TWI_APIF_bm) {
        // run user receive function after Master Write/Slave Read.
        // should get to this point after the STOP or REPSTART 
        // TWI0_SlaveAddressMatchHandler sets run_user_receive_callback_after_STOP_or_REPSTART
        // a.k.a. preventing clock streatching
        if(run_user_receive_callback_after_STOP_or_REPSTART == 1) {
            if(slave_bytesRead < TWI0_BUFFER_LENGTH) {
                twi0_slaveRxBuffer[slave_bytesRead] = '\0';
                slave_bytesRead += 1;
            }
            twi0_onSlaveRx(twi0_slaveRxBuffer, slave_bytesRead);
            slave_bytesRead = 0;
            run_user_receive_callback_after_STOP_or_REPSTART = 0;
        }

        // address match
        if(currentStatus & TWI_AP_bm) {
            TWI0_SlaveAddressMatchHandler();    
        }

        // If stop
        else {
            TWI0_SlaveStopHandler();

            // When the APIF interupt occures for a STOP we can miss an 
            // address event if we don't check the CLKHOLD bit.
            if(TWI0.SSTATUS & TWI_CLKHOLD_bm) {
                TWI0_SlaveAddressMatchHandler(); // clears CLKHOLD
            }
        }
    }
    
    // got data
    else if (currentStatus & TWI_DIF_bm) {
        if (currentStatus & TWI_COLL_bm) { // Collision flag
            slave_bytesRead = 0;
            slave_bytesWritten = 0;
            slave_bytesToWrite = 0;
            TWI0_SlaveTransactionFinished(TWI0S_RESULT_TRANSMIT_COLLISION);
            TWI0.SSTATUS = TWI_COLL_bm; // clear the collision flag
        } else {
            TWI0.SCTRLA |= (TWI_APIEN_bm | TWI_PIEN_bm); // Enable STOP interrupt
            if(TWI0.SSTATUS & TWI_DIR_bm) { // Master Read/Slave Write
                if((slave_bytesWritten > 0) && (TWI0.SSTATUS & TWI_RXACK_bm)) { // got NACK
                    TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
                    TWI0_SlaveTransactionFinished(TWI0S_RESULT_OK);
                } else { // got ACK, more data expected
                    if(slave_bytesWritten < slave_bytesToWrite) {
                        // twi0_fillSlaveTxBuffer is used to save buffer into twi0_slaveTxBuffer
                        uint8_t data = twi0_slaveTxBuffer[slave_bytesWritten];
                        TWI0.SDATA = data; // send data
                        slave_bytesWritten++;    
                        TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc; // wait for next data interrupt
                    } else { // buffer overflow
                        TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
                        TWI0_SlaveTransactionFinished(TWI0S_RESULT_BUFFER_OVERFLOW);   
                    }     
                }
            } else { // Master Write/Slave Read
                if(slave_bytesRead < TWI0_BUFFER_LENGTH) { // check for free buffer space
                    uint8_t data = TWI0.SDATA;
                    twi0_slaveRxBuffer[slave_bytesRead] = data;
                    slave_bytesRead++;
                    TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc; // Send ACK and wait for next data interrupt
                } else { // buffer overflow, send NACK and wait for next START. Set result buffer overflow
                    TWI0.SCTRLB = TWI_ACKACT_bm | TWI_SCMD_COMPTRANS_gc;
                    TWI0_SlaveTransactionFinished(TWI0S_RESULT_BUFFER_OVERFLOW);
                }
            }
        }
        // TWI0.SSTATUS = TWI_DIF_bm; // the Data Interrupt Flag bit was cleared when reading TWI0.SDATA or writing SCMD bit to TWI0.SCTRLB
    }
    
    // unexpected SSTATUS 
    else 
    {
        TWI0_SlaveTransactionFinished(TWI0S_RESULT_FAIL);
    }
}

/*************** PUBLIC ***********************************/

// Initialize TWI0 modules (bitrate, pull-up)
// if bitrate is 0 then disable twi, a normal bitrate is 100000UL, 
void twi0_init(uint32_t bitrate, TWI0_PINS_t pull_up)
{
    if (bitrate == 0) {
        if(twi_mode == TWI_MODE_DUAL) {
            twi0_slaveAddress(0); // nix the slave first
        }
        // disable twi modules (master and slave), acks, and twi interrupt
        TWI0.MCTRLA = 0x00;
        TWI0.MBAUD = 0x00;
        TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
        TWI0.SADDR = 0x00;
        TWI0.SCTRLA = 0x00;
  
        // deactivate internal pullups for twi.
        ioCntl(TWI0_SCL_PIN, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
        ioCntl(TWI0_SDA_PIN, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
        twim_mode = TWIM_MODE_UNKNOWN;
        twi_mode = TWI_MODE_UNKNOWN;
    } else {
        if(twim_mode != TWIM_MODE_UNKNOWN) { //disable twi module befor changing frequency
            TWI0.MCTRLA = 0x00;
            TWI0.MBAUD = 0x00;
            TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
            TWI0.SADDR = 0x00;
            TWI0.SCTRLA = 0x00;
        }

        // initialize state machine
        twi_mode = TWI_MODE_MASTER;
        twim_mode = TWIM_MODE_ENABLE;
        master_trans_status = TWI0M_STATUS_READY;
        master_result = TWI0M_RESULT_OK;
        twi0_protocall = TWI0_PROTOCALL_STOP & ~TWI0_PROTOCALL_REPEATEDSTART;

        // TWIn.DUALCTRL register can configure which pins are used for dual or split (master and slave)
        uint8_t temp_twiroutea = PORTMUX.TWIROUTEA & ~PORTMUX_TWI0_gm;
        PORTMUX.TWIROUTEA = temp_twiroutea | TWI0_MUX; // PORTMUX_TWI0_ALT2_gc

        ioDir(TWI0_SCL_PIN, DIRECTION_INPUT);
        ioDir(TWI0_SDA_PIN, DIRECTION_INPUT);

        // weak pullup pull-up.
        if (pull_up == TWI0_PINS_PULLUP) {
            ioCntl(TWI0_SCL_PIN, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_ENABLE, PORT_INVERT_NORMAL);
            ioCntl(TWI0_SDA_PIN, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_ENABLE, PORT_INVERT_NORMAL);
        }

        // initialize buffers
        master_bytesRead = 0;
        master_bytesWritten = 0;

        // dual contorl mode is used to split the functions, thus master on PC2 and PC3; slave on PC6,PC7
        // TWI0.DUALCTRL = TWI_ENABLE_bm; // but I want both on PC2 and PC3
        // enable twi module, acks, and twi interrupt
        TWI0.MCTRLA = TWI_RIEN_bm | TWI_WIEN_bm | TWI_ENABLE_bm;
        twi0_SetBaud(bitrate);
        TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
    }
}

// TWI Asynchronous Write Transaction, returns
// 0 .. Transaction started, check status for success
// 1 .. to much data, it did not fit in buffer
// 2 .. TWI wrong mode prob needs init
// 3 .. TWI status is busy
// 4 .. TWI not ready to start Rx/Tx
TWI0_WRT_t twi0_masterAsyncWrite(uint8_t slave_address, uint8_t *write_data, uint8_t bytes_to_write, TWI0_PROTOCALL_t send_stop)
{
    if (uart0_availableForWrite()) {
        fprintf(&uartstream0_f,"TWI=%d TWIM=%d status=%d\r\n", twi_mode, twim_mode, master_trans_status);
    }
    if (!( (twi_mode == TWI_MODE_MASTER) || (twi_mode == TWI_MODE_DUAL)) ) return TWI0_WRT_WRONG_MODE;

    if ((twim_mode != TWIM_MODE_ENABLE)) return TWI0_WRT_NOT_READY;

    if (master_trans_status != TWI0M_STATUS_READY) return TWI0_WRT_STATUS;

    if (bytes_to_write > TWI0_BUFFER_LENGTH) return TWI0_WRT_TO_MUCH_DATA;

    // Initiate the new transaction if bus is ready
    master_trans_status = TWI0M_STATUS_BUSY;
    master_result = TWI0M_RESULT_UNKNOWN;
    master_readData = NULL;

    // buffer data so wright can return without blocking. 
    // twim_mode is TWIM_MODE_TRANSMIT (e.g. !TWIM_MODE_ENABLE) until done.
    for(uint8_t i = 0; i < bytes_to_write; ++i) {
        twi0_masterBuffer[i] = write_data[i];
    }
    master_writeData = twi0_masterBuffer;
    master_bytesToWrite = bytes_to_write;
    master_bytesToRead = 0;
    master_bytesWritten = 0;
    master_bytesRead = 0;
    master_sendStop = send_stop;
    master_slaveAddress = slave_address<<1;

    // Write or Ping will send the START condition + Address + 'R/_W = 0'
    twi_mode = TWI_MODE_MASTER_TRANSMIT;
    twim_mode = TWIM_MODE_TRANSMIT;
    uint8_t writeAddress = ADD_WRITE_BIT(master_slaveAddress);
    TWI0.MADDR = writeAddress;

    twi0_error = TWI_ERROR_NONE;
    return TWI0_WRT_TRANSACTION_STARTED;
}

// TWI master write transaction status.
TWI0_WRT_STAT_t twi0_masterAsyncWrite_status(void)
{
    if (master_result == TWI0M_RESULT_UNKNOWN)
        return TWI0_WRT_STAT_BUSY;
    else if (TWI_ERROR_NONE == twi0_error)
        return TWI0_WRT_STAT_SUCCESS;
    else if (TWI_ERROR_MT_SLAVE_ADDR_NACK == twi0_error) 
        return TWI0_WRT_STAT_ADDR_NACK;
    else if (TWI_ERROR_MT_DATA_NACK == twi0_error) 
        return TWI0_WRT_STAT_DATA_NACK;
    else if (TWI_ERROR_ILLEGAL == twi0_error) 
        return TWI0_WRT_STAT_ILLEGAL;
    else 
        return 5; // can not happen
}

// TWI write with a state machine so the wait can be done elsewhere
// loop until loop_state == TWI0_LOOP_STATE_DONE, then return value is 
// 0 .. success
// 1 .. length to long for buffer
// 2 .. address send, NACK received
// 3 .. data send, NACK received
// 4 .. illegal start or stop condition
uint8_t twi0_masterWrite(uint8_t slave_address, uint8_t* write_data, uint8_t bytes_to_write, TWI0_PROTOCALL_t send_stop, TWI0_LOOP_STATE_t *loop_state)
{
    TWI0_WRT_t twi_state_machine = TWI0_WRT_TRANSACTION_STARTED;
    TWI0_WRT_STAT_t status = TWI0_WRT_STAT_SUCCESS;
    switch (*loop_state)
    {
        case TWI0_LOOP_STATE_RAW:
        case TWI0_LOOP_STATE_DONE:
            break; // there was nothing to do
        case TWI0_LOOP_STATE_ASYNC_WRT:
            twi_state_machine = twi0_masterAsyncWrite(slave_address, write_data, bytes_to_write, send_stop);
            if (twi_state_machine == TWI0_WRT_TO_MUCH_DATA) 
            {
                *loop_state = TWI0_LOOP_STATE_DONE;
                status = TWI0_WRT_STAT_BUSY; // report TWI0_WRT_STAT_BUSY (1) when TWI0_WRT_TO_MUCH_DATA occures
                break; // data did not fit in the buffer, request ignored
            }
            else if (twi_state_machine == TWI0_WRT_NOT_READY)
            {
                break; // the twi state machine is in use.
            }
            else 
            {
                *loop_state = TWI0_LOOP_STATE_STATUS_WRT;
                break; // the twi state machine was given data and made ready.
            }
        case TWI0_LOOP_STATE_STATUS_WRT:
            status = twi0_masterAsyncWrite_status();
            if (status == TWI0_WRT_STAT_BUSY)
            {
                break; // the twi state machine has the data and we are waiting for it to finish.
            }
            else
            {
                *loop_state = TWI0_LOOP_STATE_DONE;
                break; // all done
            }
        case TWI0_LOOP_STATE_INIT:
        case TWI0_LOOP_STATE_ASYNC_RD:
        case TWI0_LOOP_STATE_STATUS_RD:
            {
                *loop_state = TWI0_LOOP_STATE_DONE;
                break; // wrong state was set befor running
            }
    }
    return status; // note that TWI1_WRT_STAT_BUSY (1) is reported when TWI1_WRT_TO_MUCH_DATA occures
}

// TWI write busy-wait transaction with time to live in ticks (max 2**16), a tick is 64*256 clocks
// 0 .. success
// 1 .. length to long for buffer
// 2 .. address send, NACK received
// 3 .. data send, NACK received
// 4 .. illegal start or stop condition
// 5 .. twi out of time
uint8_t twi0_masterBlockingWrite(uint8_t slave_address, uint8_t* write_data, uint8_t bytes_to_write, TWI0_PROTOCALL_t send_stop/*, int ttl*/)
{
    uint8_t twi_wrt_code = 0;
    int ttl = 1024;
    unsigned long twi0_wrt_started_at = tickAtomic();
    TWI0_LOOP_STATE_t loop_state = TWI0_LOOP_STATE_ASYNC_WRT; // loop state is in this blocking function rather than in the main loop
    while (loop_state != TWI0_LOOP_STATE_DONE) {
        twi_wrt_code = twi0_masterWrite(slave_address, write_data, bytes_to_write, send_stop, &loop_state);
        unsigned long kRuntime = elapsed(&twi0_wrt_started_at);
        if (kRuntime > ttl) {
            return 5;
        }
    }
    return twi_wrt_code;
}

// TWI Asynchronous Read Transaction.
// 0 .. data fit in buffer, check twi0_masterAsyncRead_bytesRead for when it is done
// 1 .. data will not fit in the buffer, request ignored
// 2 .. TWI wrong mode prob needs init
// 3 .. TWI status is busy
// 4 .. TWI not ready to start Rx/Tx
// 5 .. TWI use write to ping
TWI0_RD_t twi0_masterAsyncRead(uint8_t slave_address, uint8_t bytes_to_read, TWI0_PROTOCALL_t send_stop)
{
    if (!( (twi_mode == TWI_MODE_MASTER) || (twi_mode == TWI_MODE_DUAL)) ) return TWI0_WRT_WRONG_MODE;

    if ((twim_mode != TWIM_MODE_ENABLE)) return TWI0_RD_NOT_READY;

    if (master_trans_status != TWI0M_STATUS_READY) return TWI0_RD_STATUS;

    if (bytes_to_read > TWI0_BUFFER_LENGTH) return TWI0_RD_TO_MUCH_DATA;

    if (bytes_to_read == 0) return TWI0_RD_USE_WRT_TO_PING;

    // Initiate the transaction
    master_trans_status = TWI0M_STATUS_BUSY;
    master_result = TWI0M_RESULT_UNKNOWN;
    master_writeData = NULL;

    // use the buffer to hold the data so read can return without blocking. 
    // twim_mode is TWIM_MODE_RECEIVE (e.g. !TWIM_MODE_ENABLE) until done.
    master_readData = twi0_masterBuffer;
    master_bytesToWrite = 0;
    master_bytesToRead = bytes_to_read;
    master_bytesWritten = 0;
    master_bytesRead = 0;
    master_sendStop = send_stop;
    master_slaveAddress = slave_address << 1;

    // Read will send the START condition + Address + 'R/_W = 1'
    twi_mode = TWI_MODE_MASTER_RECEIVE;
    twim_mode = TWIM_MODE_RECEIVE;
    uint8_t readAddress = ADD_READ_BIT(master_slaveAddress);
    TWI0.MADDR = readAddress;

    twi0_error = TWI_ERROR_NONE;
    return TWI0_RD_TRANSACTION_STARTED;
}

// TWI master Asynchronous Read Transaction status 
// clears the twi0_error value so that getBytes can read data NACKed befor finishing.
TWI0_RD_STAT_t twi0_masterAsyncRead_status(void)
{
    TWI_ERROR_t twi0_error_temp = TWI_ERROR_NONE;
    if (master_result == TWI0M_RESULT_UNKNOWN)
        return TWI0_RD_STAT_BUSY;
    else if (TWI_ERROR_NONE == twi0_error)
        twi0_error_temp = TWI0_RD_STAT_SUCCESS;
    else if (TWI_ERROR_MS_SLAVE_ADDR_NACK == twi0_error) 
        twi0_error_temp = TWI0_RD_STAT_ADDR_NACK;
    else if (TWI_ERROR_MS_DATA_NACK == twi0_error) 
        twi0_error_temp = TWI0_RD_STAT_DATA_NACK;
    else if (TWI_ERROR_ILLEGAL == twi0_error) 
        twi0_error_temp = TWI0_RD_STAT_ILLEGAL;
    return twi0_error_temp;
}

// TWI master Asynchronous Read Transaction get bytes. 
// Output   0 .. twi busy, read operation not complete or has error (check status)
//      1..32 .. number of bytes read
// note that a data NACK will give no output, but reading status will clear twi0_error so you can get what is available.
uint8_t twi0_masterAsyncRead_getBytes(uint8_t *read_data)
{
    if ( (master_result == TWI0M_RESULT_UNKNOWN) || (TWI_ERROR_NONE != twi0_error) )
    {
        return 0;
    }

    uint8_t bytes_read;
    if (master_bytesRead < master_bytesToRead)
    {
        bytes_read = master_bytesRead;
    }
    else
    {
        bytes_read = master_bytesToRead;
    }

    for(uint8_t i = 0; i < bytes_read; ++i)
    {
        read_data[i] = twi0_masterBuffer[i];
    }

    return bytes_read;
}

// TWI read with a state machine so the wait can be done elsewhere
// loop until loop_state == TWI0_LOOP_STATE_DONE, then return value is 
// 0 returns when somthing went wrong, was it from TWI0_RD_TO_MUCH_DATA, or check twi0_masterAsyncRead_status
// 1..32 returns the number of bytes
uint8_t twi0_masterRead(uint8_t slave_address, uint8_t* read_data, uint8_t bytes_to_read, TWI0_PROTOCALL_t send_stop, TWI0_LOOP_STATE_t *loop_state)
{
    TWI0_RD_t twi_state_machine = TWI0_RD_TRANSACTION_STARTED;
    TWI0_RD_STAT_t status = TWI0_RD_STAT_BUSY;
    uint8_t bytes_read = 0;
    switch (*loop_state)
    {
        case TWI0_LOOP_STATE_RAW:
        case TWI0_LOOP_STATE_DONE:
            break; // there was nothing to do
        case TWI0_LOOP_STATE_ASYNC_RD:
            twi_state_machine = twi0_masterAsyncRead(slave_address, bytes_to_read, send_stop);
            if (twi_state_machine == TWI0_RD_TO_MUCH_DATA) 
            {
                *loop_state = TWI0_LOOP_STATE_DONE; // done, 
                break; // read request ignored, data did not fit in the buffer.
            }
            else if (twi_state_machine == TWI0_RD_NOT_READY)
            {
                break; // but the twi state machine is in use.
            }
            else 
            {
                *loop_state = TWI0_LOOP_STATE_STATUS_RD;
                break; // machine was told to read data and made ready.
            }
        case TWI0_LOOP_STATE_STATUS_RD:
            status = twi0_masterAsyncRead_status();
            if (status == TWI0_RD_STAT_BUSY)
            {
                break; // state machine is getting the data and we are waiting for it to finish.
            }
            else if ( (status == TWI0_RD_STAT_ADDR_NACK) || (status == TWI0_RD_STAT_DATA_NACK) || (status == TWI0_RD_STAT_ILLEGAL) ) // read faild 
            {
                *loop_state = TWI0_LOOP_STATE_DONE; // done,
                break; // read failed, the status command can be used again to get the error
            }
            else
            {
                bytes_read = twi0_masterAsyncRead_getBytes(read_data);
                *loop_state = TWI0_LOOP_STATE_DONE;
                break; // all done
            }
        case TWI0_LOOP_STATE_INIT:
        case TWI0_LOOP_STATE_ASYNC_WRT:
        case TWI0_LOOP_STATE_STATUS_WRT:
            {
                *loop_state = TWI0_LOOP_STATE_DONE;
                break; // wrong state was set befor running
            }
    }
    return bytes_read;
}

// TWI read busy-wait transaction, do not use with multi-master.
// 0 returns if requeste for data will not fit in buffer
// 1..32 returns the number of bytes
uint8_t twi0_masterBlockingRead(uint8_t slave_address, uint8_t* read_data, uint8_t bytes_to_read, TWI0_PROTOCALL_t send_stop)
{
    uint8_t bytes_read = 0;
    TWI0_LOOP_STATE_t loop_state = TWI0_LOOP_STATE_ASYNC_RD; // loop state is in this blocking function rather than in the main loop
    while (loop_state != TWI0_LOOP_STATE_DONE)
    {
        bytes_read = twi0_masterRead(slave_address, read_data, bytes_to_read, send_stop, &loop_state);
    }
    return bytes_read;
}

// TWI Write and then Read with a state machine so the wait can be done elsewhere
// befor calling define a variable for the stat machine to use 
// static TWI0_LOOP_STATE_t twi0_loop_state = TWI0_LOOP_STATE_DONE;
// each time the loop_state machine is used it needs started with
// twi0_loop_state = TWI0_LOOP_STATE_ASYNC_WRT;
// when operations are finished twi0_loop_state == TWI0_LOOP_STATE_DONE 
// and the return value should have the bytes_read in lower 5 bits (0..32)
// error codes are moved to the upper 3 bits (e.g., twi_wrt_code<<5 or twi0_masterAsyncRead_status()<<5)
uint8_t twi0_masterWriteRead(uint8_t slave_address, uint8_t* write_data, uint8_t bytes_to_write, uint8_t* read_data, uint8_t bytes_to_read, TWI0_LOOP_STATE_t *loop_state)
{
    if ( (*loop_state == TWI0_LOOP_STATE_ASYNC_WRT) || (*loop_state == TWI0_LOOP_STATE_STATUS_WRT) )
    {
        uint8_t twi_wrt_code = twi0_masterWrite(slave_address, write_data, bytes_to_write, TWI0_PROTOCALL_REPEATEDSTART, loop_state);
        if ( (*loop_state == TWI0_LOOP_STATE_DONE) && (twi_wrt_code == 0) )
        {
            *loop_state = TWI0_LOOP_STATE_ASYNC_RD;
        }
        else
        {
            // twi_wrt_code may have an error in which case loop_state is set as TWI0_LOOP_STATE_DONE
            // or write is not done and we can check again after a spin through the outside loop
            return twi_wrt_code<<5; // if twi_wrt_code has an error use twi0_masterAsyncWrite_status to see it
        }
    }

    uint8_t bytes_read = twi0_masterRead(slave_address, read_data, bytes_to_read, TWI0_PROTOCALL_STOP, loop_state);
    if (*loop_state == TWI0_LOOP_STATE_DONE)
    {
        if (bytes_read)
        {
            return bytes_read;
        }
        else
        {
            return twi0_masterAsyncRead_status()<<5;
        }
    }
    else
    {
        return 0; // not done
    }
    
}

// Init slave with a valid address (0x08..0x77), otherwise slave hardware is disabled.
// Must be in master mode first, this will start dual mode where both master and slave run togather.
// Returns address if dual mode successful.
uint8_t twi0_slaveAddress(uint8_t slave)
{
    if( (twi_mode != TWI_MODE_MASTER) || (twi_mode != TWI_MODE_DUAL) )return 0; // I want a multi-master. Starting master first seems like the most straightforward logistical approach.

    if(twis_mode != TWIS_MODE_UNKNOWN) return 0;

    if( (slave>=0x8) && (slave<=0x77)) {
        twi_mode = TWI_MODE_DUAL; // DUAL mode
        twis_mode = TWIS_MODE_ENABLE;
        slave_bytesRead = 0;
        slave_bytesWritten = 0;
        slave_trans_status = TWI0S_STATUS_READY;
        slave_result = TWI0S_RESULT_UNKNOWN;
        run_user_receive_callback_after_STOP_or_REPSTART = 0;
        
        TWI0.SADDR = slave << 1;
        // enable an interrupt on the Data Interrupt Flag (DIF) from the Slave Status (TWIn.SSTATUS) register
        // enable an interrupt on the Address or Stop Interrupt Flag (APIF) from the Slave Status (TWIn.SSTATUS) register
        // allow the Address or Stop Interrupt Flag (APIF) in the Slave Status (TWIn.SSTATUS) register to be set when a Stop condition occurs
        TWI0.SCTRLA = TWI_DIEN_bm | TWI_APIEN_bm | TWI_PIEN_bm  | TWI_ENABLE_bm;
        
        // Dual Control Enable bit allows the slave to operate simultaneously with master
        TWI0.MCTRLA = TWI_ENABLE_bm;
        return slave;
    } else { // turn off all Slave Control A bits so only master hardware is running
        TWI0.SCTRLA = 0;
        twi_mode = TWI_MODE_MASTER; // back to master mode
        twis_mode = TWIS_MODE_UNKNOWN;
        twi0_onSlaveTx = twi0_transmit_default;
        twi0_onSlaveRx = twi0_receive_default;
        return 0;
    }
}

// fill twi0_slaveTxBuffer using callback returns
// 0: OK
// 1: bytes_to_send is to much for buffer, so request ignored
// 2: TWI state machine is not in slave mode, so request ignored
uint8_t twi0_fillSlaveTxBuffer(const uint8_t* slave_data, uint8_t bytes_to_send)
{
    if(TWI0_BUFFER_LENGTH < bytes_to_send)
    {
        return 1;
    }

    for(uint8_t i = 0; i < bytes_to_send; ++i)
    {
        twi0_slaveTxBuffer[i] = slave_data[i];
    }
    slave_bytesToWrite = bytes_to_send;

    return 0;
}

// record callback to use durring a slave read operation 
// a NULL pointer will use the default callback
void twi0_registerSlaveRxCallback( void (*function)(uint8_t* data, uint8_t length) )
{
    if (function == NULL )
    {
        twi0_onSlaveRx = twi0_receive_default;
    }
    else
    {
        twi0_onSlaveRx = function;
    }
}

// record callback to use before a slave write operation
// a NULL pointer will use the default callback
void twi0_registerSlaveTxCallback( void (*function)(void) )
{
    if (function == NULL )
    {
        twi0_onSlaveTx = twi0_transmit_default;
    }
    else
    {
        twi0_onSlaveTx = function;
    }
}
