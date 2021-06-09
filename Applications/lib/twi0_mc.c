/******************************************************************************
* (c) 2018 Microchip Technology Inc. and its subsidiaries.
* 
* Subject to your compliance with these terms, you may use Microchip software 
* and any derivatives exclusively with Microchip products. It is your 
* responsibility to comply with third party license terms applicable to your 
* use of third party software (including open source software) that may 
* accompany Microchip software.
*
* THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS".  NO WARRANTIES, WHETHER 
* EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY IMPLIED 
* WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS FOR A PARTICULAR 
* PURPOSE. IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, 
* PUNITIVE, INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY 
* KIND WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP 
* HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE.  TO THE 
* FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL CLAIMS IN 
* ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT OF FEES, IF ANY, 
* THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS SOFTWARE.
*
*
* 2020 Ronald Sutherland (mods for AVR128DA and my io lib)
 ****************************************************************************
 
 You have to enable interrupts in your code, somewhere before the first use of 
 the read or write functions. You also have to define a write buffer and a 
 read buffer that are passed, by pointer, to the read and write functions.

 https://www.avrfreaks.net/forum/arduino-twi-module-adapted-mega0-and-as7
 */
#include <stdbool.h>
#include <stddef.h>
#include <avr/interrupt.h>
#include "twi0_mc.h"
#include "io_enum_bsd.h"

/* Master variables */
static volatile uint8_t  master_slaveAddress;                       /*!< Slave address */
static volatile uint8_t *master_writeData;                          /*!< Data to write */
static volatile uint8_t *master_readData;                           /*!< Read data */
static volatile uint8_t  master_bytesToWrite;                       /*!< Number of bytes to write */
static volatile uint8_t  master_bytesToRead;                        /*!< Number of bytes to read */
static volatile uint8_t  master_bytesWritten;                       /*!< Number of bytes written */
static volatile uint8_t  master_bytesRead;                          /*!< Number of bytes read */
static volatile uint8_t  master_sendStop;                           /*!< To send a stop at the end of the transaction or not */
static volatile uint8_t  master_trans_status;                       /*!< Status of transaction */
static volatile uint8_t  master_result;                             /*!< Result of transaction */

/* Slave variables */
static uint8_t TWI_slaveTxBuffer[TWI0_BUFFER_LENGTH];
static uint8_t TWI_slaveRxBuffer[TWI0_BUFFER_LENGTH];
static volatile uint8_t *slave_writeData;
static volatile uint8_t  slave_bytesToWrite;
static volatile uint8_t  slave_bytesWritten;
static volatile uint8_t  slave_bytesToRead;
static volatile uint8_t  slave_bytesRead;
static volatile uint8_t  slave_trans_status; // this is does noting and can be removed
static volatile uint8_t  slave_result;
static volatile uint8_t  slave_callUserReceive;
static volatile uint8_t  slave_callUserRequest;

/* TWI module mode */
static volatile TWI_MODE_t twi_mode;

/*! \brief Initialize the TWI module as a master.
 *
 *  TWI master initialization function.
 *  Enables master read and write interrupts. 
 *  Disable the TWI module with a frequency of 0.
 *  Remember to enable interrupts globally from the main application.
 *
 *  \param frequency                    The required baud.
 */
void TWI_MasterInit(uint32_t frequency)
{
    if (frequency == 0)
    {
        // disable twi modules (master and slave), acks, and twi interrupt
        TWI0.MCTRLA = 0x00;
        TWI0.MBAUD = 0x00;
        TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
        TWI0.SADDR = 0x00;
        TWI0.SCTRLA = 0x00;
  
        // deactivate internal pullups for twi.
        ioCntl(TWI0_SCL_PIN, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
        ioCntl(TWI0_SDA_PIN, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
        twi_mode = TWI_MODE_UNKNOWN;
    }
    else
    {
        if(twi_mode != TWI_MODE_UNKNOWN) 
        { //disable twi module befor changing frequency
            TWI0.MCTRLA = 0x00;
            TWI0.MBAUD = 0x00;
            TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
            TWI0.SADDR = 0x00;
            TWI0.SCTRLA = 0x00;
        }

        // initialize state machine
        twi_mode = TWI_MODE_MASTER;

        // TWIn.DUALCTRL register can configure which pins are used for dual or split (master and slave)
        uint8_t temp_twiroutea = PORTMUX.TWIROUTEA & ~PORTMUX_TWI0_gm;
        PORTMUX.TWIROUTEA = temp_twiroutea | TWI0_MUX; // PORTMUX_TWI0_ALT2_gc

        ioDir(TWI0_SDA_PIN, DIRECTION_INPUT);
        ioDir(TWI0_SCL_PIN, DIRECTION_INPUT);

#ifdef NO_EXTERNAL_I2C_PULLUP
        ioCntl(TWI0_SDA_PIN,PORT_ISC_INTDISABLE_gc,PORT_PULLUP_ENABLE,PORT_INVERT_NORMAL);
        ioCntl(TWI0_SCL_PIN,PORT_ISC_INTDISABLE_gc,PORT_PULLUP_ENABLE,PORT_INVERT_NORMAL);
#endif

        master_bytesRead = 0;
        master_bytesWritten = 0;
        master_trans_status = TWIM_STATUS_READY;
        master_result = TWIM_RESULT_UNKNOWN;

        /* dual contorl mode is used to split the functions, thus master on PC2 and PC3; slave on PC6,PC7
           TWI0.DUALCTRL = TWI_ENABLE_bm; // but I want both on PC2 and PC3
           enable twi module, acks, and twi interrupt */
        TWI0.MCTRLA = TWI_RIEN_bm | TWI_WIEN_bm | TWI_ENABLE_bm;
        TWI_MasterSetBaud(frequency);
        TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
    }
}

void TWI_Flush(void){
    TWI0.MCTRLB |= TWI_FLUSH_bm;
}

/*! \brief Returns the TWI bus state.
 *
 *  Returns the TWI bus state (type defined in device headerfile),
 *  unknown, idle, owner or busy.
 *
 *  \param twi The TWI_Master_t struct instance.
 *
 *  \retval TWI_MASTER_BUSSTATE_UNKNOWN_gc Bus state is unknown.
 *  \retval TWI_MASTER_BUSSTATE_IDLE_gc    Bus state is idle.
 *  \retval TWI_MASTER_BUSSTATE_OWNER_gc   Bus state is owned by the master.
 *  \retval TWI_MASTER_BUSSTATE_BUSY_gc    Bus state is busy.
 */
TWI_BUSSTATE_t TWI_MasterState(void)
{
    TWI_BUSSTATE_t twi_status;
    twi_status = (TWI_BUSSTATE_t) (TWI0.MSTATUS & TWI_BUSSTATE_gm);
    return twi_status;
}


/*! \brief Returns true if transaction is ready.
 *
 *  This function returns a boolean whether the TWI Master is ready
 *  for a new transaction.
 *
 *  \param twi The TWI_Master_t struct instance.
 *
 *  \retval true  If transaction could be started.
 *  \retval false If transaction could not be started.
 */
uint8_t TWI_MasterReady(void)
{
    uint8_t twi_status = (master_trans_status & TWIM_STATUS_READY);
    return twi_status;
}

/*! \brief Set the TWI clock rate.
 *
 *  Sets the SCL clock rate used by TWI Master.
 *
 *  \param f_scl   The SCL clock rate.
 */
void TWI_MasterSetBaud(uint32_t frequency){
    uint8_t restore = TWI0.MCTRLA;
    if (restore & TWI_ENABLE_bm) {
        TWI0.MCTRLA = 0; // The TWI master should be disabled while changing the baud rate
    }

    // Use (F_CPU/(2*frequency)) - (5 + (((F_CPU / 1000000) * t_rise) / 2000)) to calculate the baud rate with t_rise (max) in ns and the frequencies in Hz
    uint16_t t_rise;
    int16_t baud;

    // The nonlinearity of the frequency coupled with the processor frequency a general offset has been calculated and tested for different frequency bands
    #if F_CPU > 16000000
    if (frequency <= 100000) {
        TWI0.CTRLA &= ~TWI_FMPEN_bm; // Disable fast mode plus
        t_rise = 1000;
        baud = (F_CPU / (2 * frequency)) - (5 + (((F_CPU / 1000000) * t_rise) / 2000)) + 6; // Offset +6
    } else if (frequency <= 400000) {
        TWI0.CTRLA &= ~TWI_FMPEN_bm; // Disable fast mode plus
        t_rise = 300;
        baud = (F_CPU / (2 * frequency)) - (5 + (((F_CPU / 1000000) * t_rise) / 2000)) + 1; // Offset +1
    } else if (frequency <= 800000) {
        TWI0.CTRLA &= ~TWI_FMPEN_bm; // Disable fast mode plus
        t_rise = 120;
        baud = (F_CPU / (2 * frequency)) - (5 + (((F_CPU / 1000000) * t_rise) / 2000));
    } else {
        TWI0.CTRLA |= TWI_FMPEN_bm; // Enable fast mode plus
        t_rise = 120;
        baud = (F_CPU / (2 * frequency)) - (5 + (((F_CPU / 1000000) * t_rise) / 2000)) - 1; // Offset -1
    }
    #else
    if (frequency <= 100000) {
        TWI0.CTRLA &= ~TWI_FMPEN_bm; // Disable fast mode plus
        t_rise = 1000;
        baud = (F_CPU / (2 * frequency)) - (5 + (((F_CPU / 1000000) * t_rise) / 2000)) + 8; // Offset +8
    } else if (frequency <= 400000) {
        TWI0.CTRLA &= ~TWI_FMPEN_bm; // Disable fast mode plus
        t_rise = 300;
        baud = (F_CPU / (2 * frequency)) - (5 + (((F_CPU / 1000000) * t_rise) / 2000)) + 1; // Offset +1
    } else if (frequency <= 800000) {
        TWI0.CTRLA &= ~TWI_FMPEN_bm; // Disable fast mode plus
        t_rise = 120;
        baud = (F_CPU / (2 * frequency)) - (5 + (((F_CPU / 1000000) * t_rise) / 2000));
    } else {
        TWI0.CTRLA |= TWI_FMPEN_bm; // Enable fast mode plus
        t_rise = 120;
        baud = (F_CPU / (2 * frequency)) - (5 + (((F_CPU / 1000000) * t_rise) / 2000)) - 1; // Offset -1
    }
    #endif

    if (baud < 1) {
        baud = 1;
    } else if (baud > 255) {
        baud = 255;
    }

    TWI0.MBAUD = (uint8_t)baud;
    if (restore & TWI_ENABLE_bm) {
        TWI0.MCTRLA  = restore;
        TWI0.MSTATUS = TWI_BUSSTATE_IDLE_gc;
    }


    
    /* the old way: f_scl is now frequency
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
    uint32_t baud = (F_CPU/(2*f_scl)) - ( (5 + ((F_CPU/1000000)*t_rise)/2000 )/2 );
    TWI0.MBAUD = (uint8_t)baud;
    */
}

/*! \brief TWI write transaction.
 *
 *  This function is TWI Master wrapper for a write-only transaction.
 *
 *  \param twi          The TWI_Master_t struct instance.
 *  \param address      Slave address.
 *  \param writeData    Pointer to data to write.
 *  \param bytesToWrite Number of data bytes to write.
 *
 *  \retval true  If transaction could be started.
 *  \retval false If transaction could not be started.
 */
uint8_t TWI_MasterWrite(uint8_t slave_address,
                     uint8_t *write_data,
                     uint8_t bytes_to_write,
                     uint8_t send_stop)
{
    return TWI_MasterWriteRead(slave_address, 
                        write_data, 
                        bytes_to_write, 
                        0,
                        send_stop);
}


/*! \brief TWI read transaction.
 *
 *  This function is a TWI Master wrapper for read-only transaction.
 *
 *  \param twi            The TWI_Master_t struct instance.
 *  \param address        The slave address.
 *  \param bytesToRead    The number of bytes to read.
 *
 *  \retval true  If transaction could be started.
 *  \retval false If transaction could not be started.
 */
uint8_t TWI_MasterRead(uint8_t slave_address,
                    uint8_t *read_data,
                    uint8_t bytes_to_read,
                    uint8_t send_stop)
{
    master_readData = read_data;

    uint8_t bytes_read = TWI_MasterWriteRead(slave_address, 
                                          0, 
                                          0, 
                                          bytes_to_read,
                                          send_stop);
    return bytes_read;
}


/*! \brief TWI write and/or read transaction.
 *
 *  This function is a TWI Master write and/or read transaction. The function
 *  can be used to both write and/or read bytes to/from the TWI Slave in one
 *  transaction.
 *
 *  \param twi            The TWI_Master_t struct instance.
 *  \param address        The slave address.
 *  \param writeData      Pointer to data to write.
 *  \param bytesToWrite   Number of bytes to write.
 *  \param bytesToRead    Number of bytes to read.
 *
 *  \retval 0:success
 *  \retval 1:data too long to fit in transmit buffer
 *  \retval 2:received NACK on transmit of address
 *  \retval 3:received NACK on transmit of data
 *  \retval 4:other error
 */
uint8_t TWI_MasterWriteRead(uint8_t slave_address,
                         uint8_t *write_data,
                         uint8_t bytes_to_write,
                         uint8_t bytes_to_read,
                         uint8_t send_stop)
{
    if(twi_mode != TWI_MODE_MASTER) {
        return false;
    }

    /* Initiate transaction if bus is ready. */
    if (master_trans_status == TWIM_STATUS_READY) {
        
        master_trans_status = TWIM_STATUS_BUSY;
        master_result = TWIM_RESULT_UNKNOWN;

        master_writeData = write_data;

        master_bytesToWrite = bytes_to_write;
        master_bytesToRead = bytes_to_read;
        master_bytesWritten = 0;
        master_bytesRead = 0;
        master_sendStop = send_stop;
        master_slaveAddress = slave_address << 1;

trigger_action:

        /* If write command, send the START condition + Address +
           'R/_W = 0'
         */
        if (master_bytesToWrite > 0) {
            twi_mode = TWI_MODE_MASTER_TRANSMIT;
            uint8_t writeAddress = ADD_WRITE_BIT(master_slaveAddress);
            TWI0.MADDR = writeAddress;
        }

        /* If read command, send the START condition + Address +
           'R/_W = 1'
         */
        else if (master_bytesToRead > 0) {
            twi_mode = TWI_MODE_MASTER_RECEIVE;
            uint8_t readAddress = ADD_READ_BIT(master_slaveAddress);
            TWI0.MADDR = readAddress;
        }

        else if (master_bytesToWrite == 0 && master_bytesToRead == 0) {
            twi_mode = TWI_MODE_MASTER_TRANSMIT;
            uint8_t writeAddress = ADD_WRITE_BIT(master_slaveAddress);
            TWI0.MADDR = writeAddress;
        }

        /* Arduino requires blocking function */
        while(master_result == TWIM_RESULT_UNKNOWN) {}

        // in case of arbitration lost, retry sending
        if (master_result == TWIM_RESULT_ARBITRATION_LOST) {
            goto trigger_action; // return 4;
        }

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
}

/*! \brief TWI transaction finished handler.
 *
 *  Prepares module for new transaction.
 *
 *  \param result  The result of the operation.
 */
void TWI_MasterTransactionFinished(uint8_t result)
{
    master_result = result;
    master_trans_status = TWIM_STATUS_READY;
    twi_mode = TWI_MODE_MASTER;
}

// used to initalize the slave Transmit function in case it is not used.
void TWI_transmit_default(void)
{
    // In a real callback, the data to send needs to be copied from a local buffer.
    // uint8_t return_code = TWI_fillSlaveTxBuffer(localBuffer, localBufferLength);

    // this default puts a NUL byte in the buffer and bypasses the provided function
    slave_bytesToWrite = 1;
    TWI_slaveTxBuffer[0] = 0x00;
    return;
}

// used to initalize the slave Receive function in case is not used.
void TWI_receive_default(uint8_t *data, uint8_t length)
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
typedef void (*PointerToReceive)(uint8_t *, uint8_t);
static PointerToTransmit TWI_onSlaveTransmit = TWI_transmit_default;
static PointerToReceive TWI_onSlaveReceive = TWI_receive_default;

/*! \brief Initialize the TWI module as a slave.
 *
 *  TWI slave initialization function.
 *  Enables slave address/stop and data interrupts.
 *  Assigns slave's own address.
 *  Remember to enable interrupts globally from the main application.
 *
 *  \param address   The address (range: 0x8..0x77) TWI Slave will use.
 */
uint8_t TWI_SlaveInit(uint8_t address)
{
    if(twi_mode != TWI_MODE_UNKNOWN) return 0;
    if( (address>=0x8) && (address<=0x77))
    {
        twi_mode = TWI_MODE_SLAVE;

        slave_bytesRead = 0;
        slave_bytesWritten = 0;
        slave_trans_status = TWIS_STATUS_READY;
        slave_result = TWIS_RESULT_UNKNOWN;
        slave_callUserRequest = 0;
        slave_callUserReceive = 0;

        TWI0.SADDR = address << 1;
        // enable an interrupt on the Data Interrupt Flag (DIF) from the Slave Status (TWIn.SSTATUS) register
        // enable an interrupt on the Address or Stop Interrupt Flag (APIF) from the Slave Status (TWIn.SSTATUS) register
        // allow the Address or Stop Interrupt Flag (APIF) in the Slave Status (TWIn.SSTATUS) register to be set when a Stop condition occurs
        TWI0.SCTRLA = TWI_DIEN_bm | TWI_APIEN_bm | TWI_PIEN_bm | TWI_ENABLE_bm;

        // Dual Control Enable bit mask which should allow the slave to operate simultaneously with master
        TWI0.MCTRLA = TWI_ENABLE_bm;
        return address;
    }
    else
    {
        // turn off all Slave Control A bits so only master hardware is running
        TWI0.SCTRLA = 0;
        twi_mode = TWI_MODE_UNKNOWN;
        TWI_onSlaveTransmit = TWI_transmit_default;
        TWI_onSlaveReceive = TWI_receive_default;
        return 0;
    }
}

/*! \brief TWI slave address interrupt handler.
 *
 *  This is the slave address match handler that takes care of responding to
 *  being addressed by a master
 *
 */
void TWI_SlaveAddressMatchHandler(){
    slave_trans_status = TWIS_STATUS_BUSY;
    slave_result = TWIS_RESULT_UNKNOWN;
    
    /* Send ACK, wait for data interrupt */
    TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;    
    
    /* If Master Read/Slave Write */
    if(TWI0.SSTATUS & TWI_DIR_bm){
        slave_bytesWritten = 0;
        /* Call user function  */
        TWI_onSlaveTransmit(); // slave_bytesToWrite is saved durring TWI_fillSlaveTxBuffer
        twi_mode = TWI_MODE_SLAVE_TRANSMIT;
    } 
    /* If Master Write/Slave Read */
    else {
        slave_bytesRead = 0;
        slave_callUserReceive = 1;
        twi_mode = TWI_MODE_SLAVE_RECEIVE;
    }
}

/*! \brief TWI slave transaction finished handler.
 *
 *  Prepares module for new transaction.
 *
 *  \param result  Save the result of the operation.
 */
void TWI_SlaveTransactionFinished(uint8_t result)
{
    TWI0.SCTRLA |= (TWI_APIEN_bm | TWI_PIEN_bm);
    twi_mode = TWI_MODE_SLAVE;
    slave_result = result;
    slave_trans_status = TWIS_STATUS_READY;
}

/* \brief TWI slave transmit transaction helper.
 * Desc     fill TWI_slaveTxBuffer, normaly need done durring transmit event.
 * Input    data location and length.
 * Output   0: OK, 1: bytes_to_send is to much for buffer, 2: TWI state machine is not in slave mode
 */
uint8_t TWI_fillSlaveTxBuffer(const uint8_t *slave_data, uint8_t bytes_to_send)
{
    if(TWI0_BUFFER_LENGTH < bytes_to_send)
    {
        return 1;
    }

    for(uint8_t i = 0; i < bytes_to_send; ++i)
    {
        TWI_slaveTxBuffer[i] = slave_data[i];
    }
    slave_bytesToWrite = bytes_to_send;

    return 0;
}


/* 
 * Function twi_attachSlaveRxEvent
 * Desc     sets (or registers callback) function used when a slave read operation occurs.
 * Input    record callback to use durring a slave read operation
 * Output   none
 */
void TWI_attachSlaveRxEvent( void (*function)(uint8_t *data, uint8_t length) ) {
    if (function == NULL )
    {
        TWI_onSlaveReceive = TWI_receive_default;
    }
    else
    {
        TWI_onSlaveReceive = function;;
    }
}

/* 
 * Function twi_attachSlaveTxEvent
 * Desc     record callback to use before a slave write operation
 * Input    callback function, a NULL pointer will use the default callback.
 * Output   none
 */
void TWI_attachSlaveTxEvent( void (*function)(void) ) {
    if (function == NULL )
    {
        TWI_onSlaveTransmit = TWI_transmit_default;
    }
    else
    {
        TWI_onSlaveTransmit = function;
    }
}

ISR(TWI0_TWIM_vect) {
    uint8_t currentStatus = TWI0.MSTATUS;

    /* If arbitration lost or bus error. */
    if ((currentStatus & TWI_ARBLOST_bm) || (currentStatus & TWI_BUSERR_bm)) {
        uint8_t currentStatus = TWI0.MSTATUS;

        if (currentStatus & TWI_BUSERR_bm) { /* bus error */
            master_result = TWIM_RESULT_BUS_ERROR;
        } else { /* arbitration lost */
            master_result = TWIM_RESULT_ARBITRATION_LOST;
        }

        /* Clear all flags, abort operation */
        TWI0.MSTATUS = currentStatus;

        /* Wait for a new operation */
        twi_mode = TWI_MODE_MASTER;
        master_trans_status = TWIM_STATUS_READY;
    }

    /* If master write interrupt. */
    else if (currentStatus & TWI_WIF_bm) {
        /* Local variables used in if tests to avoid compiler warning. */
        uint8_t local_bytesToWrite  = master_bytesToWrite;
        uint8_t local_bytesToRead   = master_bytesToRead;

        /* If NOT acknowledged (NACK) by slave cancel the transaction. */
        if (TWI0.MSTATUS & TWI_RXACK_bm) {
            if(master_sendStop) {
                TWI0.MCTRLB = TWI_MCMD_STOP_gc;
            } else {
                TWI0.MCTRLB = TWI_MCMD_REPSTART_gc;
            }
            TWI_MasterTransactionFinished(TWIM_RESULT_NACK_RECEIVED);
        }

        /* acknowledged (ACK) by slave, If more bytes to write then send next. */
        else if (master_bytesWritten < local_bytesToWrite) {
            uint8_t data = master_writeData[master_bytesWritten];
            TWI0.MDATA = data;
            master_bytesWritten++;
        }

        /* acknowledged (ACK) by slave without more data to send. Maybe bytes to read?
        * i2c read frame looks like: START condition + (Address + 'R/_W = 1')
        */
        else if (master_bytesRead < local_bytesToRead) {
            twi_mode = TWI_MODE_MASTER_RECEIVE;
            uint8_t readAddress = ADD_READ_BIT(master_slaveAddress);
            TWI0.MADDR = readAddress;
        }

        /* transaction finished, send STOP or REPSTART condition and set RESULT OK. */
        else {
            if(master_sendStop) {
                TWI0.MCTRLB = TWI_MCMD_STOP_gc;
            } else {
                TWI0.MCTRLB = TWI_MCMD_REPSTART_gc;
            }
            TWI_MasterTransactionFinished(TWIM_RESULT_OK);
        }
    }

    /* If master read interrupt. */
    else if (currentStatus & TWI_RIF_bm) {
        /* Fetch data if bytes to be read. */
        if (master_bytesRead < master_bytesToRead) {
            uint8_t data = TWI0.MDATA;
            master_readData[master_bytesRead] = data;
            master_bytesRead++;
        }

        /* If buffer overflow, issue NACK/STOP and BUFFER_OVERFLOW condition. */
        else {
            if(master_sendStop) {
                TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_STOP_gc;
            } else {
                TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_REPSTART_gc;
            }
            
            TWI_MasterTransactionFinished(TWIM_RESULT_BUFFER_OVERFLOW);
            master_bytesToRead = 0;
            return;
        }

        /* Local variable used in if test to avoid compiler warning. */
        uint8_t bytesToRead = master_bytesToRead;

        /* If more bytes to read, issue ACK and start a byte read. */
        if (master_bytesRead < bytesToRead) {
            TWI0.MCTRLB = TWI_MCMD_RECVTRANS_gc;
        }

        /* If transaction finished, transaction done, issue NACK (1 to ACKACT bit) and STOP or REPSTART condition. */
        else {
            if(master_sendStop) {
                TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_STOP_gc;
            } else {
                TWI0.MCTRLB = TWI_ACKACT_bm | TWI_MCMD_REPSTART_gc;
            }
            
            TWI_MasterTransactionFinished(TWIM_RESULT_OK);
        }
    }

    /* If unexpected state. */
    else {
        TWI_MasterTransactionFinished(TWIM_RESULT_FAIL);
    }
}

ISR(TWI0_TWIS_vect){
    uint8_t currentStatus = TWI0.SSTATUS;
    
    /* If bus error */
    if(currentStatus & TWI_BUSERR_bm){
        slave_bytesRead = 0;
        slave_bytesWritten = 0;
        slave_bytesToWrite = 0;
        TWI_SlaveTransactionFinished(TWIS_RESULT_BUS_ERROR);
    }
    
    /* If Address or Stop */
    else if(currentStatus & TWI_APIF_bm){
        
        /* Call user onReceive function if end of Master Write/Slave Read.
         * This should be hit when there is a STOP or REPSTART 
         */
        if(slave_callUserReceive == 1){
            TWI_onSlaveReceive(TWI_slaveRxBuffer, slave_bytesRead);
            slave_callUserReceive = 0;
        }
        
        /* If address match */
        if(currentStatus & TWI_AP_bm){
            TWI_SlaveAddressMatchHandler();    
        }
        
        /* If stop */
        else {
            /* Clear APIF, don't ACK or NACK */
            TWI0.SSTATUS = TWI_APIF_bm;
            
            TWI_SlaveTransactionFinished(TWIS_RESULT_OK);
    
            
            /* If CLKHOLD is high, we have missed an address match 
              from a fast start after stop. 
              Because the flag is shared we need to handle this here.
            */
            if(TWI0.SSTATUS & TWI_CLKHOLD_bm){
                
                /* CLKHOLD will be cleared by servicing the address match */
                TWI_SlaveAddressMatchHandler();
            }
        }
    }
    
    /* If Data Interrupt */
    else if (currentStatus & TWI_DIF_bm){
        
        /* If collision flag is raised, slave transmit unsuccessful */
        if (currentStatus & TWI_COLL_bm){
            slave_bytesRead = 0;
            slave_bytesWritten = 0;
            slave_bytesToWrite = 0;
            TWI_SlaveTransactionFinished(TWIS_RESULT_TRANSMIT_COLLISION);
        } 
        
        /* Otherwise, normal data interrupt */
        else {
            /* Enable stop interrupt */
            TWI0.SCTRLA |= (TWI_APIEN_bm | TWI_PIEN_bm);

            /* If Master Read/Slave Write */
            if(TWI0.SSTATUS & TWI_DIR_bm) {
                /* If NACK, slave write transaction finished */
                if((slave_bytesWritten > 0) && (TWI0.SSTATUS & TWI_RXACK_bm)){
                    TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
                    TWI_SlaveTransactionFinished(TWIS_RESULT_OK);
                } else { /* ACK, master expects more data */
                    if(slave_bytesWritten < slave_bytesToWrite){
                        uint8_t data = slave_writeData[slave_bytesWritten];
                        TWI0.SDATA = data;
                        slave_bytesWritten++;    
                        
                        /* Send data, wait for data interrupt */
                        TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
                    } else { /* buffer overflow */
                        TWI0.SCTRLB = TWI_SCMD_COMPTRANS_gc;
                        TWI_SlaveTransactionFinished(TWIS_RESULT_BUFFER_OVERFLOW);
                    }
                }
            } else { /* If Master Write/Slave Read */
                /* If free space in buffer */
                if(slave_bytesRead < slave_bytesToRead) {
                    uint8_t data = TWI0.SDATA; /* Fetch data */
                    TWI_slaveRxBuffer[slave_bytesRead] = data;
                    slave_bytesRead++;
                    /* Send ACK and wait for data interrupt */
                    TWI0.SCTRLB = TWI_SCMD_RESPONSE_gc;
                } else { /* If buffer empty, send NACK and wait for next START. */
                    TWI0.SCTRLB = TWI_ACKACT_bm | TWI_SCMD_COMPTRANS_gc;
                    TWI_SlaveTransactionFinished(TWIS_RESULT_BUFFER_OVERFLOW);
                }
            }
        }
    }
    
    /* If unexpected state */
    else {
        TWI_SlaveTransactionFinished(TWIS_RESULT_FAIL);
    }
}

