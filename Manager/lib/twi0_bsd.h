#pragma once

#define TWI0_BUFFER_LENGTH 32

// TWI0 master(dual)/slave on PA2:PA3/PC2:PC3 use PORTMUX.TWIROUTEA |= PORTMUX_TWI0_DEFAULT_gc
// TWI0 master(dual)/slave on PA2:PA3/PC6:PC7 use PORTMUX.TWIROUTEA |= PORTMUX_TWI0_ALT1_gc
// TWI0 master(dual)/slave on PC2:PC3/PC6:PC7 use PORTMUX.TWIROUTEA |= PORTMUX_TWI0_ALT2_gc
#define TWI0_MUX PORTMUX_TWI0_ALT2_gc

/*! Transaction status defines. */
#define TWI0M_STATUS_READY              0
#define TWI0M_STATUS_BUSY               1

/* Transaction status defines.*/
#define TWI0S_STATUS_READY                0
#define TWI0S_STATUS_BUSY                 1

typedef enum TWI0_PINS_enum
{
    TWI0_PINS_FLOATING,
    TWI0_PINS_PULLUP
} TWI0_PINS_t;

// TWI master write attempted.
typedef enum TWI0_WRT_enum
{
    TWI0_WRT_TRANSACTION_STARTED, // Transaction started
    TWI0_WRT_TO_MUCH_DATA, // to much data
    TWI0_WRT_NOT_READY // TWI state machine not ready for use
} TWI0_WRT_t;

// TWI master write transaction status.
typedef enum TWI0_WRT_STAT_enum
{
    TWI0_WRT_STAT_SUCCESS, // success
    TWI0_WRT_STAT_BUSY, // twi busy, write operation not complete
    TWI0_WRT_STAT_ADDR_NACK, // address send, NACK received
    TWI0_WRT_STAT_DATA_NACK, // data send, NACK received
    TWI0_WRT_STAT_ILLEGAL // illegal start or stop condition
} TWI0_WRT_STAT_t;

// TWI master read attempted.
typedef enum TWI0_RD_enum
{
    TWI0_RD_TRANSACTION_STARTED, // Transaction started
    TWI0_RD_TO_MUCH_DATA, // to much data
    TWI0_RD_NOT_READY // TWI state machine not ready for use
} TWI0_RD_t;

// TWI master read transaction status.
typedef enum TWI0_RD_STAT_enum
{
    TWI0_RD_STAT_SUCCESS, // success
    TWI0_RD_STAT_BUSY, // twi busy, read operation not complete
    TWI0_RD_STAT_ADDR_NACK, // address send, NACK received
    TWI0_RD_STAT_DATA_NACK, // data receive, NACK received
    TWI0_RD_STAT_ILLEGAL // illegal start or stop condition
} TWI0_RD_STAT_t;

// Use a repeated start to hold (lock) the bus for a future transaction.
// The ISR will generate a start that locks the bus
typedef enum TWI0_PROTOCALL_enum
{
    TWI0_PROTOCALL_STOP, // Stop sending and unlock the bus
    TWI0_PROTOCALL_REPEATEDSTART // lock the bus and wait for more to send
} TWI0_PROTOCALL_t;

typedef enum TWI0_LOOP_STATE_enum
{
    TWI0_LOOP_STATE_RAW, // TWI has not been used yet, set INIT to start, this state will do nothing
    TWI0_LOOP_STATE_DONE, // TWI read/write is done so do nothing
    TWI0_LOOP_STATE_INIT, // set up static buffers for the next states to use
    TWI0_LOOP_STATE_ASYNC_WRT, // this can fail if data does not fit buffer, and needs to retry if TWI state machine is not ready
    TWI0_LOOP_STATE_STATUS_WRT, // the TWI state machine will give a status when it has finished
    TWI0_LOOP_STATE_ASYNC_RD, // this can fail if data does not fit buffer, and needs to retry if TWI state machine is not ready
    TWI0_LOOP_STATE_STATUS_RD // the TWI state machine will have a status when it has finished
} TWI0_LOOP_STATE_t;

// Master result
typedef enum TWI0M_RESULT_enum
{
    TWI0M_RESULT_UNKNOWN,
    TWI0M_RESULT_OK,
    TWI0M_RESULT_BUFFER_OVERFLOW,
    TWI0M_RESULT_ARBITRATION_LOST,
    TWI0M_RESULT_BUS_ERROR,
    TWI0M_RESULT_NACK_RECEIVED,
    TWI0M_RESULT_FAIL
} TWI0M_RESULT_t;

// Slave result
typedef enum TWI0S_RESULT_enum
{
    TWI0S_RESULT_UNKNOWN,
    TWI0S_RESULT_OK,
    TWI0S_RESULT_BUFFER_OVERFLOW,
    TWI0S_RESULT_TRANSMIT_COLLISION,
    TWI0S_RESULT_BUS_ERROR,
    TWI0S_RESULT_FAIL,
    TWI0S_RESULT_ABORTED
} TWI0S_RESULT_t;

void twi0_init(uint32_t bitrate, TWI0_PINS_t pull_up);

TWI0_WRT_t twi0_masterAsyncWrite(uint8_t slave_address, uint8_t *write_data, uint8_t bytes_to_write, TWI0_PROTOCALL_t send_stop);
TWI0_WRT_STAT_t twi0_masterAsyncWrite_status(void);
uint8_t twi0_masterWrite(uint8_t slave_address, uint8_t* write_data, uint8_t bytes_to_write, TWI0_PROTOCALL_t send_stop, TWI0_LOOP_STATE_t *loop_state);
uint8_t twi0_masterBlockingWrite(uint8_t slave_address, uint8_t* write_data, uint8_t bytes_to_write, TWI0_PROTOCALL_t send_stop);

TWI0_RD_t twi0_masterAsyncRead(uint8_t slave_address, uint8_t bytes_to_read, TWI0_PROTOCALL_t send_stop);
TWI0_RD_STAT_t twi0_masterAsyncRead_status(void);
uint8_t twi0_masterAsyncRead_getBytes(uint8_t *read_data);
uint8_t twi0_masterRead(uint8_t slave_address, uint8_t* read_data, uint8_t bytes_to_read, TWI0_PROTOCALL_t send_stop, TWI0_LOOP_STATE_t *loop_state);
uint8_t twi0_masterBlockingRead(uint8_t slave_address, uint8_t* read_data, uint8_t bytes_to_read, TWI0_PROTOCALL_t send_stop);

uint8_t twi0_masterWriteRead(uint8_t slave_address, uint8_t* write_data, uint8_t bytes_to_write, uint8_t* read_data, uint8_t bytes_to_read, TWI0_LOOP_STATE_t *loop_state);

uint8_t twi0_slaveAddress(uint8_t slave);
uint8_t twi0_fillSlaveTxBuffer(const uint8_t* slave_data, uint8_t bytes_to_send);
void twi0_registerSlaveRxCallback( void (*function)(uint8_t* data, uint8_t length) );
void twi0_registerSlaveTxCallback( void (*function)(void) );

