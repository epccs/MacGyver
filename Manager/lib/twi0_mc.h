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
 *****************************************************************************/
#ifndef TWI_DRIVER_H
#define TWI_DRIVER_H

#include "avr/io.h"

#define TWI0_BUFFER_LENGTH 32

// TWI0 master(dual)/slave on PA2:PA3/PC2:PC3 use PORTMUX.TWIROUTEA |= PORTMUX_TWI0_DEFAULT_gc
// TWI0 master(dual)/slave on PA2:PA3/PC6:PC7 use PORTMUX.TWIROUTEA |= PORTMUX_TWI0_ALT1_gc
// TWI0 master(dual)/slave on PC2:PC3/PC6:PC7 use PORTMUX.TWIROUTEA |= PORTMUX_TWI0_ALT2_gc
#define TWI0_MUX PORTMUX_TWI0_ALT2_gc
#define TWI0_SDA_PIN MCU_IO_MVIO_SDA0
#define TWI0_SCL_PIN MCU_IO_MVIO_SCL0

// no correction today, needs 32k crystal and a clue
#define F_CPU_CORRECTED F_CPU

/*! Transaction status defines. */
#define TWIM_STATUS_READY              0
#define TWIM_STATUS_BUSY               1

/* Transaction status defines.*/
#define TWIS_STATUS_READY                0
#define TWIS_STATUS_BUSY                 1

/*! Transaction result enumeration. */
typedef enum TWIM_RESULT_enum {
    TWIM_RESULT_UNKNOWN          = (0x00<<0),
    TWIM_RESULT_OK               = (0x01<<0),
    TWIM_RESULT_BUFFER_OVERFLOW  = (0x02<<0),
    TWIM_RESULT_ARBITRATION_LOST = (0x03<<0),
    TWIM_RESULT_BUS_ERROR        = (0x04<<0),
    TWIM_RESULT_NACK_RECEIVED    = (0x05<<0),
    TWIM_RESULT_FAIL             = (0x06<<0),
} TWIM_RESULT_t;

/* Transaction result enumeration */
typedef enum TWIS_RESULT_enum {
    TWIS_RESULT_UNKNOWN            = (0x00<<0),
    TWIS_RESULT_OK                 = (0x01<<0),
    TWIS_RESULT_BUFFER_OVERFLOW    = (0x02<<0),
    TWIS_RESULT_TRANSMIT_COLLISION = (0x03<<0),
    TWIS_RESULT_BUS_ERROR          = (0x04<<0),
    TWIS_RESULT_FAIL               = (0x05<<0),
    TWIS_RESULT_ABORTED            = (0x06<<0),
} TWIS_RESULT_t;

/*! TWI Modes */
typedef enum TWI_MODE_enum {
    TWI_MODE_UNKNOWN = 0,
    TWI_MODE_MASTER = 1,
    TWI_MODE_SLAVE = 2,
    TWI_MODE_MASTER_TRANSMIT = 3,
    TWI_MODE_MASTER_RECEIVE = 4,
    TWI_MODE_SLAVE_TRANSMIT = 5,
    TWI_MODE_SLAVE_RECEIVE = 6
} TWI_MODE_t;

/*! For adding R/_W bit to address */
#define ADD_READ_BIT(address)    (address | 0x01)
#define ADD_WRITE_BIT(address)  (address & ~0x01)

void TWI_MasterInit(uint32_t frequency);
void TWI_Flush(void);
TWI_BUSSTATE_t TWI_MasterState(void);
uint8_t TWI_MasterReady(void);
void TWI_MasterSetBaud(uint32_t frequency);
uint8_t TWI_MasterWrite(uint8_t slave_address,
                     uint8_t *write_data,
                     uint8_t bytes_to_write,
                     uint8_t send_stop);
uint8_t TWI_MasterRead(uint8_t slave_address,
                    uint8_t* read_data,
                    uint8_t bytes_to_read,
                    uint8_t send_stop);
uint8_t TWI_MasterWriteRead(uint8_t slave_address,
                         uint8_t *write_data,
                         uint8_t bytes_to_write,
                         uint8_t bytes_to_read,
                         uint8_t send_stop);
void TWI_MasterTransactionFinished(uint8_t result);

uint8_t TWI_SlaveInit(uint8_t address);
void TWI_SlaveAddressMatchHandler(void);
void TWI_SlaveTransactionFinished(uint8_t result);
uint8_t TWI_fillSlaveTxBuffer(const uint8_t* slave_data, uint8_t bytes_to_send);
void TWI_attachSlaveRxEvent( void (*function)(uint8_t* data, uint8_t length) );
void TWI_attachSlaveTxEvent( void (*function)(void) );


#endif /* TWI_DRIVER_H */