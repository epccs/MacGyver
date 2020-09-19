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

TODO: This was used on a m324pb but it will need a lot of updates to work with a AVR-DA.

Multi-Master can lock up when blocking functions are used. Also the master will get a NACK 
if it tries to acceess a slave on the same state machine. Note: an AVR128DA has two ISR driven
state machines, one for the master and another for the slave but it is not clear to me if
they can share the same IO hardware wihtout locking up. The AVR128DA famly has alternat 
IO hardware, the master ISR can be on one set of hardware while the slave ISR is on the other.
*/

#include <stdbool.h>
#include <avr/interrupt.h>
#include <util/twi.h>
#include "io_enum_bsd.h"
#include "twi0_bsd.h"

/* TWI master status */
#define TWIM_STATUS_READY              0
#define TWIM_STATUS_BUSY               1

/* TWI slave status */
#define TWIS_STATUS_READY                0
#define TWIS_STATUS_BUSY                 1

/*! For adding R/_W bit to address */
#define ADD_READ_BIT(address)    (address | 0x01)
#define ADD_WRITE_BIT(address)  (address & ~0x01)

static volatile uint8_t twi0_slave_read_write;

typedef enum TWI_STATE_enum {
    TWI_STATE_READY, // TWI state machine ready for use
    TWI_STATE_MASTER_RECEIVER, // TWI state machine is master receiver
    TWI_STATE_MASTER_TRANSMITTER, // TWI state machine is master transmitter
    TWI_STATE_SLAVE_RECEIVER, // TWI state machine is slave receiver
    TWI_STATE_SLAVE_TRANSMITTER  // TWI state machine is slave transmitter
} TWI_STATE_t;

static volatile TWI_STATE_t twi0_MastSlav_RxTx_state;

/* TWI modes modules mode */
typedef enum TWI_MODE_enum {
    TWI_MODE_UNKNOWN = 0,
    TWI_MODE_MASTER = 1,
    TWI_MODE_SLAVE = 2,
    TWI_MODE_MASTER_TRANSMIT = 3,
    TWI_MODE_MASTER_RECEIVE = 4,
    TWI_MODE_SLAVE_TRANSMIT = 5,
    TWI_MODE_SLAVE_RECEIVE = 6
} TWI_MODE_t;

static volatile TWI_MODE_t twi_mode;

typedef enum TWI_ACK_enum {
    TWI_ACK,
    TWI_NACK
} TWI_ACK_t;

static volatile TWI0_PROTOCALL_t twi0_protocall; 

typedef enum TWI_ERROR_enum {
    TWI_ERROR_ILLEGAL = TW_BUS_ERROR, // illegal start or stop condition
    TWI_ERROR_MT_SLAVE_ADDR_NACK = TW_MT_SLA_NACK, // Master Transmiter SLA+W transmitted, NACK received 
    TWI_ERROR_MT_DATA_NACK = TW_MT_DATA_NACK, // Master Transmiter data transmitted, NACK received
    TWI_ERROR_ARBITRATION_LOST = TW_MT_ARB_LOST, // Master Transmiter arbitration lost in SLA+W or data
    TWI_ERROR_MS_SLAVE_ADDR_NACK = TW_MR_SLA_NACK, // Master Receiver SLA+W transmitted, NACK received 
    TWI_ERROR_MS_DATA_NACK = TW_MR_DATA_NACK, // Master Receiver data received, NACK returned ()
    // TW_MR_ARB_LOST is done with TW_MT_ARB_LOST
    TWI_ERROR_NONE = 0xFF // No errors
} TWI_ERROR_t;

static volatile TWI_ERROR_t twi0_error;

// used to initalize the slave Transmit functions in case they are not used.
void twi0_transmit_default(void)
{
    // In a real callback, the data to send needs to be copied from a local buffer.
    // uint8_t return_code = twi0_fillSlaveTxBuffer(localBuffer, localBufferLength);
    return;
}

typedef void (*PointerToTransmit)(void);

// used to initalize the slave Receive functions in case they are not used.
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

static uint8_t twi0_masterBuffer[TWI0_BUFFER_LENGTH];
static volatile uint8_t twi0_masterBufferIndex;
static volatile uint8_t twi0_masterBufferLength;
static register8_t  master_slaveAddress;
static register8_t *master_writeData;
static register8_t *master_readData;
static register8_t  master_bytesToWrite;
static register8_t  master_bytesToRead;
static register8_t  master_bytesWritten;
static register8_t  master_bytesRead;
static register8_t  master_sendStop;
static register8_t  master_trans_status;
static register8_t  master_result;   

static uint8_t twi0_slaveTxBuffer[TWI0_BUFFER_LENGTH];
static volatile uint8_t twi0_slaveTxBufferIndex;
static volatile uint8_t twi0_slaveTxBufferLength;
typedef void (*PointerToReceive)(uint8_t*, uint8_t);
static PointerToTransmit twi0_onSlaveTx = twi0_transmit_default;
static PointerToReceive twi0_onSlaveRx = twi0_receive_default;
static uint8_t (*TWI_onSlaveTransmit)(void) __attribute__((unused));
static void (*TWI_onSlaveReceive)(int) __attribute__((unused));
static register8_t* slave_writeData;
static register8_t* slave_readData;
static register8_t  slave_bytesToWrite;
static register8_t  slave_bytesWritten;
static register8_t  slave_bytesToRead;
static register8_t  slave_bytesRead;
static register8_t  slave_trans_status;
static register8_t  slave_result;
static register8_t  slave_callUserReceive;
static register8_t  slave_callUserRequest;

// enable interleaving buffer for R-Pi Zero in header
static uint8_t twi0_slaveRxBufferA[TWI0_BUFFER_LENGTH];
#ifdef TWI0_SLAVE_RX_BUFFER_INTERLEAVING
static uint8_t twi0_slaveRxBufferB[TWI0_BUFFER_LENGTH];
#endif

static uint8_t *twi0_slaveRxBuffer;
static volatile uint8_t twi0_slaveRxBufferIndex;

// Set the BAUD bit field in the TWIn.MBAUD
void TWI_SetBaud(uint32_t f_scl)
{

    uint16_t t_rise; // is determined by the bus impedance and pull-up, this is only a ball-park.
    
    if(f_scl < 200000)
    {
        f_scl = 100000; // 100kHz
        t_rise = 1000; // 1000ns, must be bellow 4700ns for equation 2 in DS
    }
    else if (f_scl < 800000)
    {
        f_scl = 400000; // 400kHz
        t_rise = 300; // 300ns, must be bellow 1300ns for equation 2 in DS
    } 
    else if (f_scl < 1200000)
    {
        f_scl = 1000000; // 1MHz
        t_rise = 120; // 120ns, must be bellow 500ns for equation 2 in DS
    } 
    else 
    {
        f_scl = 100000;
        t_rise = 1000; 
    }
    
    // from equation 2 DS40002183B-page 401
    uint32_t baud = (F_CPU/(2*f_scl)) - ( (5 + (((F_CPU/1000)/1000)*t_rise)/1000 )/2 );
    TWI0.MBAUD = (uint8_t)baud;

}

void TWI_MasterTransactionFinished(uint8_t result)
{
    master_result = result;
    master_trans_status = TWIM_STATUS_READY;
    twi_mode = TWI_MODE_MASTER;
}

// TWI0 Master event
ISR(TWI0_TWIM_vect){
    uint8_t currentStatus = TWI0.MSTATUS;

    // arbitration lost or bus error.
    if ((currentStatus & TWI_ARBLOST_bm) || (currentStatus & TWI_BUSERR_bm)) 
    {
        uint8_t currentStatus = TWI0.MSTATUS;

        if (currentStatus & TWI_BUSERR_bm) 
        {
            master_result = TWIM_RESULT_BUS_ERROR;
        }
        else 
        {
            master_result = TWIM_RESULT_ARBITRATION_LOST;
        }

        TWI0.MSTATUS = currentStatus; // clear operation

        twi_mode = TWI_MODE_MASTER;
        master_trans_status = TWIM_STATUS_READY;
    }

    // master write 
    else if (currentStatus & TWI_WIF_bm) 
    {
        uint8_t bytesToWrite  = master_bytesToWrite;
        uint8_t bytesToRead   = master_bytesToRead;

        // If missing acknowledge (NACK) stop
        if (TWI0.MSTATUS & TWI_RXACK_bm) 
        {
            if(master_sendStop)
            {
                TWI0.MCTRLB = TWI_MCMD_STOP_gc;
            } 
            else 
            {
                TWI0.MCTRLB = TWI_MCMD_REPSTART_gc;

            }
            TWI_MasterTransactionFinished(TWIM_RESULT_NACK_RECEIVED);
        }

        // If more to write
        else if (master_bytesWritten < bytesToWrite) 
        {
            uint8_t data = master_writeData[master_bytesWritten];
            TWI0.MDATA = data;
            master_bytesWritten++;
        }

        // If read is next, send START condition + Address + 'R/_W = 1'
        else if (master_bytesRead < bytesToRead) 
        {
            twi_mode = TWI_MODE_MASTER_RECEIVE;
            uint8_t readAddress = ADD_READ_BIT(master_slaveAddress);
            TWI0.MADDR = readAddress;
        }

        // If finished, send ACK/STOP or setup repeated start
        else
        {
            if(master_sendStop)
            {
                TWI0.MCTRLB = TWI_MCMD_STOP_gc;
            } 
            else 
            {
                TWI0.MCTRLB = TWI_MCMD_REPSTART_gc;
            }
            TWI_MasterTransactionFinished(TWIM_RESULT_OK);
        }
    }

    //master read 
    else if (currentStatus & TWI_RIF_bm) 
    {
        if (master_bytesRead < master_bytesToRead) 
        {
            uint8_t data = TWI0.MDATA;
            master_readData[master_bytesRead] = data;
            master_bytesRead++;
        }
        else { // to many bytes
            if(master_sendStop){
                TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_STOP_gc;
            } 
            else 
            {
                TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_REPSTART_gc;
            }
            
            TWI_MasterTransactionFinished(TWIM_RESULT_BUFFER_OVERFLOW);
            master_bytesToRead = 0;
            return;
        }

        uint8_t bytesToRead = master_bytesToRead;

        if (master_bytesRead < bytesToRead) {
            TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc; // issue ACK to start next byte read.
        }
        else {
            if(master_sendStop)
            {
                TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_STOP_gc;
            } 
            else 
            {
                TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_REPSTART_gc;
            }
            
            TWI_MasterTransactionFinished(TWIM_RESULT_OK);
        }
    }

    // unexpected 
    else {
        TWI_MasterTransactionFinished(TWIM_RESULT_FAIL);
    }
}

void TWI_SlaveAddressMatchHandler(){
    slave_trans_status = TWIS_STATUS_BUSY;
    slave_result = TWIS_RESULT_UNKNOWN;
    
    // ACK, data should be on the way
    TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;    
    
    if(TWI0.SSTATUS & TWI_DIR_bm) // Master Read or Slave Write
    { 
        slave_bytesWritten = 0;
        /* Call user function  */
        slave_bytesToWrite = TWI_onSlaveTransmit();    
        twi_mode = TWI_MODE_SLAVE_TRANSMIT;
    } 
    else // If Master Write/Slave Read
    {
        slave_bytesRead = 0;
        slave_callUserReceive = 1;
        twi_mode = TWI_MODE_SLAVE_RECEIVE;
    }
}

void TWI_SlaveStopHandler()
{
    // Clear Address/Stop Interrupt Flag, don't ACK or NACK
    TWI0.SSTATUS = TWI_APIF_bm;
    TWI_SlaveTransactionFinished(TWIS_RESULT_OK);
}

// TWI0 Slave event
ISR(TWI0_TWIS_vect)
{
    uint8_t currentStatus = TWI0.SSTATUS;
    
    //bus error
    if(currentStatus & TWI_BUSERR_bm)
    {
        slave_bytesRead = 0;
        slave_bytesWritten = 0;
        slave_bytesToWrite = 0;
        TWI_SlaveTransactionFinished(TWIS_RESULT_BUS_ERROR);
    }
    
    // Address or Stop
    else if(currentStatus & TWI_APIF_bm)
    {
        // run user receive function after Master Write/Slave Read.
        // should get to this point after the STOP or REPSTART 
        // TWI_SlaveAddressMatchHandler sets slave_callUserReceive
        if(slave_callUserReceive == 1)
        {
            TWI_onSlaveReceive(slave_bytesRead);
            slave_callUserReceive = 0;
        }

        // address match
        if(currentStatus & TWI_AP_bm)
        {
            TWI_SlaveAddressMatchHandler();    
        }

        // If stop
        else 
        {
            TWI_SlaveStopHandler();

            // When the APIF interupt occures for a STOP we can miss an 
            // address event if we don't check the CLKHOLD bit.
            if(TWI0.SSTATUS & TWI_CLKHOLD_bm)
            {    
                TWI_SlaveAddressMatchHandler(); // clears CLKHOLD
            }
        }
    }
    
    // got data
    else if (currentStatus & TWI_DIF_bm)
    {
        // Collision flag
        if (currentStatus & TWI_COLL_bm)
        {
            slave_bytesRead = 0;
            slave_bytesWritten = 0;
            slave_bytesToWrite = 0;
            TWI_SlaveTransactionFinished(TWIS_RESULT_TRANSMIT_COLLISION);
        } 
        else 
        {
            TWI0.SCTRLA |= (TWI_APIEN_bm | TWI_PIEN_bm); // Enable STOP interrupt
            if(TWI0.SSTATUS & TWI_DIR_bm) // Master Read/Slave Write
            {
                if((slave_bytesWritten > 0) && (TWI0.SSTATUS & TWI_RXACK_bm)) // got NACK
                {
                    TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
                    TWI_SlaveTransactionFinished(TWIS_RESULT_OK);
                }
                else // got ACK, more data expected
                {        
                    if(slave_bytesWritten < slave_bytesToWrite){
                        uint8_t data = slave_writeData[slave_bytesWritten];
                        TWI0.SDATA = data; // send data
                        slave_bytesWritten++;    
                        TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc; // wait for next data interrupt
                    } 
                    
                    /* If buffer overflow */
                    else {
                        TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
                        TWI_SlaveTransactionFinished(TWIS_RESULT_BUFFER_OVERFLOW);
                        
                    }
                    
                        
                }
            }
            else // Master Write/Slave Read
            {
                TWI_SlaveReadHandler();
            }    
        }
    }
    
    // unexpected SSTATUS 
    else 
    {
        TWI_SlaveTransactionFinished(TWIS_RESULT_FAIL);
    }
}

/*
ISR(TWI0_vect)
{
    switch(TWSR0 & TW_STATUS_MASK) // TW_STATUS can be used for part with one TWI
    {
        // Illegal start or stop condition
        case TW_BUS_ERROR:
            twi0_error = TWI_ERROR_ILLEGAL;
            twi0_stop_condition();
            break;

        // Master start condition transmitted
        case TW_START:
        case TW_REP_START:
            TWDR0 = twi0_slave_read_write;
            twi0_acknowledge(TWI_ACK);
            break;

        // Master Transmitter SLA+W transmitted, ACK received
        case TW_MT_SLA_ACK:
        case TW_MT_DATA_ACK:

            if(twi0_masterBufferIndex < twi0_masterBufferLength)
            {
                TWDR0 = twi0_masterBuffer[twi0_masterBufferIndex++];
                twi0_acknowledge(TWI_ACK);
            }
            else
            {
                if (twi0_protocall & TWI0_PROTOCALL_STOP)
                    twi0_stop_condition();
                else 
                {
                    // Generate the START but set flag for the next transaction.
                    twi0_protocall |= TWI0_PROTOCALL_REPEATEDSTART;
                    TWCR0 = (1<<TWINT) | (1<<TWSTA)| (1<<TWEN) ;
                    twi0_MastSlav_RxTx_state = TWI_STATE_READY;
                }
            }
            break;

        // Master Transmier SLA+W transmitted, NACK received
        case TW_MT_SLA_NACK:
            twi0_error = TWI_ERROR_MT_SLAVE_ADDR_NACK; 
            twi0_stop_condition();
            break;

        // Master Transmier data transmitted, NACK received
        case TW_MT_DATA_NACK:
            twi0_error = TWI_ERROR_MT_DATA_NACK;
            twi0_stop_condition();
            break;

        // Master Transmier/Reciver arbitration lost in SLA+W or data
        case TW_MT_ARB_LOST: // same as TW_MR_ARB_LOST
            twi0_error = TWI_ERROR_ARBITRATION_LOST;
            twi0_ready_bus();
            break;

        // Master Reciver Slave SLA+R transmitted, NACK received
        case TW_MR_SLA_NACK:
            twi0_error = TWI_ERROR_MS_SLAVE_ADDR_NACK;
            twi0_stop_condition();
            break;

        // Master Reciver data received, ACK returned
        case TW_MR_DATA_ACK:
            twi0_masterBuffer[twi0_masterBufferIndex++] = TWDR0;
        case TW_MR_SLA_ACK:  // SLA+R transmitted, ACK received
            if(twi0_masterBufferIndex < twi0_masterBufferLength)
            {
                twi0_acknowledge(TWI_ACK);
            }
            else
            {
                twi0_acknowledge(TWI_NACK);
            }
            break;
        
        // Master Reciver data received, NACK returned
        case TW_MR_DATA_NACK:
            twi0_masterBuffer[twi0_masterBufferIndex++] = TWDR0;
            if (twi0_masterBufferIndex < twi0_masterBufferLength) // master returns nack to slave
                twi0_error = TWI_ERROR_MS_DATA_NACK; // but master has done the nack to soon 
            if (twi0_protocall & TWI0_PROTOCALL_STOP)
                twi0_stop_condition();
            else 
            {
                // Generate the START but set flag for the next transaction.
                twi0_protocall |= TWI0_PROTOCALL_REPEATEDSTART;
                TWCR0 = (1<<TWINT) | (1<<TWSTA)| (1<<TWEN) ;
                twi0_MastSlav_RxTx_state = TWI_STATE_READY;
            }    
            break;

        // Slave SLA+W received, ACK returned
        case TW_SR_SLA_ACK:
        case TW_SR_GCALL_ACK:
        case TW_SR_ARB_LOST_SLA_ACK: 
        case TW_SR_ARB_LOST_GCALL_ACK:
            twi0_MastSlav_RxTx_state = TWI_STATE_SLAVE_RECEIVER;
            twi0_slaveRxBufferIndex = 0;
            twi0_acknowledge(TWI_ACK);
            break;

        // Slave data received, ACK returned
        case TW_SR_DATA_ACK: 
        case TW_SR_GCALL_DATA_ACK: 
            if(twi0_slaveRxBufferIndex < TWI0_BUFFER_LENGTH)
            {
                twi0_slaveRxBuffer[twi0_slaveRxBufferIndex++] = TWDR0;
                twi0_acknowledge(TWI_ACK);
            }
            else
            {
                twi0_acknowledge(TWI_NACK);
            }
            break;

        // Slave Data received, return NACK
        case TW_SR_DATA_NACK:       
        case TW_SR_GCALL_DATA_NACK: 
            twi0_acknowledge(TWI_NACK);
            break;

        // Slave stop or repeated start condition received while selected
        case TW_SR_STOP:
            twi0_ready_bus();
            if(twi0_slaveRxBufferIndex < TWI0_BUFFER_LENGTH)
            {
                twi0_slaveRxBuffer[twi0_slaveRxBufferIndex] = '\0';
            }
            twi0_onSlaveRx(twi0_slaveRxBuffer, twi0_slaveRxBufferIndex);
            #ifdef TWI0_SLAVE_RX_BUFFER_INTERLEAVING
            if (twi0_slaveRxBuffer == twi0_slaveRxBufferA) 
            {
                twi0_slaveRxBuffer = twi0_slaveRxBufferB;
            }
            else
            {
                twi0_slaveRxBuffer = twi0_slaveRxBufferA;
            }
            #endif
            twi0_slaveRxBufferIndex = 0;
            break;

        // Slave SLA+R received, ACK returned
        case TW_ST_SLA_ACK: 
        case TW_ST_ARB_LOST_SLA_ACK:
            twi0_MastSlav_RxTx_state = TWI_STATE_SLAVE_TRANSMITTER;
            twi0_slaveTxBufferIndex = 0;
            twi0_slaveTxBufferLength = 0; 
            twi0_onSlaveTx(); // use twi0_fillSlaveTxBuffer(bytes, length) in callback
            if(0 == twi0_slaveTxBufferLength) // default callback does not set this
            {
                twi0_slaveTxBufferLength = 1;
                twi0_slaveTxBuffer[0] = 0x00;
            }
        case TW_ST_DATA_ACK:
            TWDR0 = twi0_slaveTxBuffer[twi0_slaveTxBufferIndex++];
            if(twi0_slaveTxBufferIndex < twi0_slaveTxBufferLength)
            {
                twi0_acknowledge(TWI_ACK);
            }
            else
            {
                twi0_acknowledge(TWI_NACK);
            }
            break;

        // Slave data transmitted, ACK or NACK received
        case TW_ST_LAST_DATA:
        case TW_ST_DATA_NACK:
            twi0_acknowledge(TWI_ACK);
            twi0_MastSlav_RxTx_state = TWI_STATE_READY;
            break;

        // Some ISR events get ignored
        case TW_NO_INFO: 
            break;
    }
}
*/

/*************** PUBLIC ***********************************/

// Initialize TWI0 modules (bitrate, pull-up)
// if bitrate is 0 then disable twi, a normal bitrate is 100000UL, 
void twi0_init(uint32_t bitrate, TWI0_PINS_t pull_up)
{
    if (bitrate == 0)
    {
        // disable twi modules (master and slave), acks, and twi interrupt
        TWI0.MCTRLA = 0x00;
        TWI0.MBAUD = 0x00;
        TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
        TWI0.SADDR = 0x00;
        TWI0.SCTRLA = 0x00;
  
        // deactivate internal pullups for twi.
        ioCntl(MCU_IO_SCL0, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
        ioCntl(MCU_IO_SDA0, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
        twi_mode = TWI_MODE_UNKNOWN;
    }
    else
    {
        if(twi_mode != TWI_MODE_UNKNOWN) return;

        // start with interleaving buffer A (B is an optional buffer if it is enabled)
        twi0_slaveRxBuffer = twi0_slaveRxBufferA;

        // initialize state machine
        twi0_MastSlav_RxTx_state = TWI_STATE_READY;
        twi_mode = TWI_MODE_MASTER;
        twi0_protocall = TWI0_PROTOCALL_STOP & ~TWI0_PROTOCALL_REPEATEDSTART;

        // use default pins PA2, PA3, PC2, PC3
        PORTMUX.TWIROUTEA |= PORTMUX_TWI0_DEFAULT_gc;

        ioDir(MCU_IO_SCL0, DIRECTION_INPUT);
        ioDir(MCU_IO_SDA0, DIRECTION_INPUT);

        // weak pullup pull-up.
        if (pull_up == TWI0_PINS_PULLUP)
        {
            ioCntl(MCU_IO_SCL0, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_ENABLE, PORT_INVERT_NORMAL);
            ioCntl(MCU_IO_SDA0, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_ENABLE, PORT_INVERT_NORMAL);
        }

        master_trans_status = TWIM_STATUS_READY;

        // enable twi module, acks, and twi interrupt
        TWI0.MCTRLA = TWI_RIEN_bm | TWI_WIEN_bm | TWI_ENABLE_bm;
        TWI_SetBaud(bitrate);
        TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
    }
}

// TWI Asynchronous Write Transaction, returns
// 0 .. Transaction started, check status for success
// 1 .. to much data, it did not fit in buffer
// 2 .. TWI state machine not ready for use
TWI0_WRT_t twi0_masterAsyncWrite(uint8_t slave_address, uint8_t *write_data, uint8_t bytes_to_write, TWI0_PROTOCALL_t send_stop)
{
    if(twi_mode != TWI_MODE_MASTER) return TWI0_WRT_NOT_READY;

    if(bytes_to_write > TWI0_BUFFER_LENGTH) return TWI0_WRT_TO_MUCH_DATA;

    /*Initiate transaction if bus is ready. */
    if (master_trans_status == TWIM_STATUS_READY) {
        
        master_trans_status = TWIM_STATUS_BUSY;
        master_result = TWIM_RESULT_UNKNOWN;

        master_writeData = write_data;

        master_bytesToWrite = bytes_to_write;
        master_bytesToRead = 0;
        master_bytesWritten = 0;
        master_bytesRead = 0;
        master_sendStop = send_stop;
        master_slaveAddress = slave_address<<1;

        // Write command, send the START condition + Address + 'R/_W = 0'
        if (master_bytesToWrite > 0) {
            twi_mode = TWI_MODE_MASTER_TRANSMIT;
            uint8_t writeAddress = ADD_WRITE_BIT(master_slaveAddress);
            TWI0.MADDR = writeAddress;
        }

        // Read command, send the START condition + Address + 'R/_W = 1'
        else if (master_bytesToRead > 0) {
            twi_mode = TWI_MODE_MASTER_RECEIVE;
            uint8_t readAddress = ADD_READ_BIT(master_slaveAddress);
            TWI0.MADDR = readAddress;
        }

        // ping
        else if (master_bytesToWrite == 0 && master_bytesToRead == 0) {
            twi_mode = TWI_MODE_MASTER_TRANSMIT;
            uint8_t writeAddress = ADD_WRITE_BIT(master_slaveAddress);
            TWI0.MADDR = writeAddress;
        }

        twi0_MastSlav_RxTx_state = TWI_STATE_MASTER_TRANSMITTER;
        twi0_error = TWI_ERROR_NONE;


        // non-blocking read should wait for 
        //if (master_result == TWIM_RESULT_UNKNOWN) return TWI0_WRT_TRANSACTION_STARTED;




        // in case of arbitration lost
        if (master_result == TWIM_RESULT_ARBITRATION_LOST) return TWI0_WRT_NOT_READY;

        uint8_t ret = 0;
        if (master_bytesToRead > 0) {
            // return bytes really read
            ret = master_bytesRead;
        } else {
            // return 0 if success, >0 otherwise (follow classic AVR conventions)
            switch (master_result) {
                case TWIM_RESULT_OK:
                    ret = 0;
                    break;
                case TWIM_RESULT_BUFFER_OVERFLOW:
                    ret = 1;
                    break;
                case TWIM_RESULT_NACK_RECEIVED:
                    ret = 3;
                    break;
                default:
                    ret = 4;
                    break;
            }
        }

        return ret;
    } else {
        return 1;
    }

    // old code from 328p
    uint8_t i;

    if(bytes_to_write > TWI0_BUFFER_LENGTH)
    {
        return TWI0_WRT_TO_MUCH_DATA;
    }
    else
    {    
        if(twi0_MastSlav_RxTx_state != TWI_STATE_READY)
        {
            return TWI0_WRT_NOT_READY;
        }
        else // do TWI0_WRT_TRANSACTION_STARTED
        {
            twi0_MastSlav_RxTx_state = TWI_STATE_MASTER_TRANSMITTER;
            twi0_error = TWI_ERROR_NONE;
            twi0_masterBufferIndex = 0;
            twi0_masterBufferLength = bytes_to_write;
            for(i = 0; i < bytes_to_write; ++i)
            {
                twi0_masterBuffer[i] = write_data[i];
            }
        
            // build SLA+W, slave device address + write bit
            twi0_slave_read_write = slave_address << 1;
            twi0_slave_read_write += TW_WRITE;
        
            return TWI0_WRT_TRANSACTION_STARTED;
        }
    }
}

// TWI master write transaction status.
TWI0_WRT_STAT_t twi0_masterAsyncWrite_status(void)
{
    if (TWI_STATE_MASTER_TRANSMITTER == twi0_MastSlav_RxTx_state)
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

// TWI write busy-wait transaction, do not use with multi-master.
// 0 .. success
// 1 .. length to long for buffer
// 2 .. address send, NACK received
// 3 .. data send, NACK received
// 4 .. illegal start or stop condition
uint8_t twi0_masterBlockingWrite(uint8_t slave_address, uint8_t* write_data, uint8_t bytes_to_write, TWI0_PROTOCALL_t send_stop)
{
    uint8_t twi_wrt_code = 0;
    TWI0_LOOP_STATE_t loop_state = TWI0_LOOP_STATE_ASYNC_WRT; // loop state is in this blocking function rather than in the main loop
    while (loop_state != TWI0_LOOP_STATE_DONE)
    {
        twi_wrt_code = twi0_masterWrite(slave_address, write_data, bytes_to_write, send_stop, &loop_state);
    }
    return twi_wrt_code;
}

// TWI Asynchronous Read Transaction.
// 0 .. data fit in buffer, check twi0_masterAsyncRead_bytesRead for when it is done
// 1 .. data will not fit in the buffer, request ignored
// 2 .. TWI state machine not ready for use
TWI0_RD_t twi0_masterAsyncRead(uint8_t slave_address, uint8_t bytes_to_read, TWI0_PROTOCALL_t send_stop)
{
    if(twi_mode != TWI_MODE_MASTER) return TWI0_WRT_NOT_READY;

    if(bytes_to_read > TWI0_BUFFER_LENGTH) return TWI0_WRT_TO_MUCH_DATA;

    /*Initiate transaction if bus is ready. */
    if (master_trans_status == TWIM_STATUS_READY) {
        
        master_trans_status = TWIM_STATUS_BUSY;
        master_result = TWIM_RESULT_UNKNOWN;

        master_writeData = write_data;

        master_bytesToWrite = bytes_to_write;
        master_bytesToRead = 0;
        master_bytesWritten = 0;
        master_bytesRead = bytes_to_read;
        master_sendStop = send_stop;
        master_slaveAddress = slave_address<<1;

        // Write command, send the START condition + Address + 'R/_W = 0'
        if (master_bytesToWrite > 0) {
            twi_mode = TWI_MODE_MASTER_TRANSMIT;
            uint8_t writeAddress = ADD_WRITE_BIT(master_slaveAddress);
            TWI0.MADDR = writeAddress;
        }

        // Read command, send the START condition + Address + 'R/_W = 1'
        else if (master_bytesToRead > 0) {
            twi_mode = TWI_MODE_MASTER_RECEIVE;
            uint8_t readAddress = ADD_READ_BIT(master_slaveAddress);
            TWI0.MADDR = readAddress;
        }

        // ping
        else if (master_bytesToWrite == 0 && master_bytesToRead == 0) {
            twi_mode = TWI_MODE_MASTER_TRANSMIT;
            uint8_t writeAddress = ADD_WRITE_BIT(master_slaveAddress);
            TWI0.MADDR = writeAddress;
        }

        twi0_MastSlav_RxTx_state = TWI_STATE_MASTER_TRANSMITTER;
        twi0_error = TWI_ERROR_NONE;


        // non-blocking read should wait for 
        //if (master_result == TWIM_RESULT_UNKNOWN) return TWI0_WRT_TRANSACTION_STARTED;




        // in case of arbitration lost
        if (master_result == TWIM_RESULT_ARBITRATION_LOST) return TWI0_WRT_NOT_READY;

        uint8_t ret = 0;
        if (master_bytesToRead > 0) {
            // return bytes really read
            ret = master_bytesRead;
        } else {
            // return 0 if success, >0 otherwise (follow classic AVR conventions)
            switch (master_result) {
                case TWIM_RESULT_OK:
                    ret = 0;
                    break;
                case TWIM_RESULT_BUFFER_OVERFLOW:
                    ret = 1;
                    break;
                case TWIM_RESULT_NACK_RECEIVED:
                    ret = 3;
                    break;
                default:
                    ret = 4;
                    break;
            }
        }

        return ret;
    } else {
        return 1;
    }

    // old code
    if(bytes_to_read > TWI0_BUFFER_LENGTH)
    {
        return TWI0_RD_TO_MUCH_DATA; //chk
    }
    else
    {
        if (TWI_STATE_READY != twi0_MastSlav_RxTx_state)
        {
            return TWI0_RD_NOT_READY;
        }
        else
        {
            twi0_MastSlav_RxTx_state = TWI_STATE_MASTER_RECEIVER;
            if (send_stop == TWI0_PROTOCALL_STOP)
            {
                twi0_protocall |= TWI0_PROTOCALL_STOP;
            }
            else
            {
                twi0_protocall &= ~TWI0_PROTOCALL_STOP;
            }
            twi0_error = TWI_ERROR_NONE;

            twi0_masterBufferIndex = 0;

            // set NACK when the _next_ to last byte received. 
            twi0_masterBufferLength = bytes_to_read-1; 

            // build SLA+R, slave device address + r bit
            twi0_slave_read_write = slave_address << 1;
            twi0_slave_read_write += TW_READ;


            // Check if ISR has done the I2C START
            if (twi0_protocall & TWI0_PROTOCALL_REPEATEDSTART)
            {
                uint8_t local_twi_protocall = twi0_protocall;
                local_twi_protocall &= ~TWI0_PROTOCALL_REPEATEDSTART;
                twi0_protocall = local_twi_protocall;
                do 
                {
                    TWDR0 = twi0_slave_read_write;
                } while(TWCR0 & (1<<TWWC));
                TWCR0 = (1<<TWINT) | (1<<TWEA) | (1<<TWEN) | (1<<TWIE);    // enable INTs, but not START
            }
            else
            {
                // send start condition
                TWCR0 = (1<<TWEN) | (1<<TWIE) | (1<<TWEA) | (1<<TWINT) | (1<<TWSTA);
            }
            return TWI0_RD_TRANSACTION_STARTED;
        }
    }
}

// TWI master Asynchronous Read Transaction status.
TWI0_RD_STAT_t twi0_masterAsyncRead_status(void)
{
    if (TWI_STATE_MASTER_RECEIVER == twi0_MastSlav_RxTx_state)
        return TWI0_RD_STAT_BUSY;
    else if (TWI_ERROR_NONE == twi0_error)
        return TWI0_RD_STAT_SUCCESS;
    else if (TWI_ERROR_MS_SLAVE_ADDR_NACK == twi0_error) 
        return TWI0_RD_STAT_ADDR_NACK;
    else if (TWI_ERROR_MS_DATA_NACK == twi0_error) 
        return TWI0_RD_STAT_DATA_NACK;
    else if (TWI_ERROR_ILLEGAL == twi0_error) 
        return TWI0_RD_STAT_ILLEGAL;
    else 
        return 5; // can not happen
}

// TWI master Asynchronous Read Transaction get bytes.
// Output   0 .. twi busy, read operation not complete or has error (check status)
//      1..32 .. number of bytes read
uint8_t twi0_masterAsyncRead_getBytes(uint8_t *read_data)
{
    if ( (twi0_MastSlav_RxTx_state == TWI_STATE_MASTER_RECEIVER) && (TWI_ERROR_NONE != twi0_error) )
    {
        return 0;
    }

    uint8_t bytes_read = twi0_masterBufferLength+1;
    if (twi0_masterBufferIndex < bytes_read )
    {
        bytes_read = twi0_masterBufferIndex;
    }

    uint8_t i;
    for(i = 0; i < bytes_read; ++i)
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

// set valid slave address (0x8..0x77) 
// return address if set
uint8_t twi0_slaveAddress(uint8_t slave)
{
    if( (slave>=0x8) && (slave<=0x77))
    {
        // TWAR0 is Slave Address Register TWA[6..0] in bits 7..1 TWGCE in bit 0
        //       TWGCE bit is for General Call Recognition
        TWAR0 = slave << 1; 
        return slave;
    }
    else
    {
        TWAR0 = 0; 
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
  
    if(TWI_STATE_SLAVE_TRANSMITTER != twi0_MastSlav_RxTx_state)
    {
        return 2;
    }
  
    twi0_slaveTxBufferLength = bytes_to_send;
    for(uint8_t i = 0; i < bytes_to_send; ++i)
    {
        twi0_slaveTxBuffer[i] = slave_data[i];
    }
  
    return 0;
}

// record callback to use durring a slave read operation 
// a NULL pointer will use the default callback
void twi0_registerSlaveRxCallback( void (*function)(uint8_t*, uint8_t) )
{
    if (function == ((void *)0) )
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
    if (function == ((void *)0) )
    {
        twi0_onSlaveTx = twi0_transmit_default;
    }
    else
    {
        twi0_onSlaveTx = function;
    }
}
