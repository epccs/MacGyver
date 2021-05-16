#pragma once

#define TWI1_BUFFER_LENGTH 32

// Some SMBus devices (Raspberry Pi Zero) can not handle clock stretching.
// An interleaving receive buffer allwows the callback to save a pointer
// to the buffer and swap to another buffer in ISR thread.
// The main thread loop has to notice and process the twi buffer befor 
// the next transaction is done.  
//#define TWI1_SLAVE_RX_BUFFER_INTERLEAVING

typedef enum TWI1_PINS_enum
{
    TWI1_PINS_FLOATING,
    TWI1_PINS_PULLUP
} TWI1_PINS_t;

// TWI master write attempted.
typedef enum TWI1_WRT_enum
{
    TWI1_WRT_TRANSACTION_STARTED, // Transaction started
    TWI1_WRT_TO_MUCH_DATA, // to much data
    TWI1_WRT_NOT_READY // TWI state machine not ready for use
} TWI1_WRT_t;

// TWI master write transaction status.
typedef enum TWI1_WRT_STAT_enum
{
    TWI1_WRT_STAT_SUCCESS, // success
    TWI1_WRT_STAT_BUSY, // twi busy, write operation not complete
    TWI1_WRT_STAT_ADDR_NACK, // address send, NACK received
    TWI1_WRT_STAT_DATA_NACK, // data send, NACK received
    TWI1_WRT_STAT_ILLEGAL // illegal start or stop condition
} TWI1_WRT_STAT_t;

// TWI master read attempted.
typedef enum TWI1_RD_enum
{
    TWI1_RD_TRANSACTION_STARTED, // Transaction started
    TWI1_RD_TO_MUCH_DATA, // to much data
    TWI1_RD_NOT_READY // TWI state machine not ready for use
} TWI1_RD_t;

// TWI master read transaction status.
typedef enum TWI1_RD_STAT_enum
{
    TWI1_RD_STAT_SUCCESS, // success
    TWI1_RD_STAT_BUSY, // twi busy, read operation not complete
    TWI1_RD_STAT_ADDR_NACK, // address send, NACK received
    TWI1_RD_STAT_DATA_NACK, // data receive, NACK received
    TWI1_RD_STAT_ILLEGAL // illegal start or stop condition
} TWI1_RD_STAT_t;

// Use a repeated start to hold (lock) the bus for a future transaction.
// The ISR will generate a start that locks the bus
typedef enum TWI1_PROTOCALL_enum
{
    TWI1_PROTOCALL_STOP, // Stop sending and unlock the bus
    TWI1_PROTOCALL_REPEATEDSTART // lock the bus and wait for more to send
} TWI1_PROTOCALL_t;

typedef enum TWI1_LOOP_STATE_enum
{
    TWI1_LOOP_STATE_RAW, // TWI has not been used yet, set INIT to start, this state will do nothing
    TWI1_LOOP_STATE_DONE, // TWI read/write is done so do nothing
    TWI1_LOOP_STATE_INIT, // set up static buffers for the next states to use
    TWI1_LOOP_STATE_ASYNC_WRT, // this can fail if data does not fit buffer, and needs to retry if TWI state machine is not ready
    TWI1_LOOP_STATE_STATUS_WRT, // the TWI state machine will give a status when it has finished
    TWI1_LOOP_STATE_ASYNC_RD, // this can fail if data does not fit buffer, and needs to retry if TWI state machine is not ready
    TWI1_LOOP_STATE_STATUS_RD // the TWI state machine will have a status when it has finished
} TWI1_LOOP_STATE_t;

// Master result
typedef enum TWI1M_RESULT_enum
{
    TWI1M_RESULT_UNKNOWN,
    TWI1M_RESULT_OK,
    TWI1M_RESULT_BUFFER_OVERFLOW,
    TWI1M_RESULT_ARBITRATION_LOST,
    TWI1M_RESULT_BUS_ERROR,
    TWI1M_RESULT_NACK_RECEIVED,
    TWI1M_RESULT_FAIL
} TWI1M_RESULT_t;

// Slave result
typedef enum TWI1S_RESULT_enum
{
    TWI1S_RESULT_UNKNOWN,
    TWI1S_RESULT_OK,
    TWI1S_RESULT_BUFFER_OVERFLOW,
    TWI1S_RESULT_TRANSMIT_COLLISION,
    TWI1S_RESULT_BUS_ERROR,
    TWI1S_RESULT_FAIL,
    TWI1S_RESULT_ABORTED
} TWI1S_RESULT_t;

void twi1_init(uint32_t bitrate, TWI1_PINS_t pull_up);

TWI1_WRT_t twi1_masterAsyncWrite(uint8_t slave_address, uint8_t *write_data, uint8_t bytes_to_write, TWI1_PROTOCALL_t send_stop);
TWI1_WRT_STAT_t twi1_masterAsyncWrite_status(void);
uint8_t twi1_masterWrite(uint8_t slave_address, uint8_t* write_data, uint8_t bytes_to_write, TWI1_PROTOCALL_t send_stop, TWI1_LOOP_STATE_t *loop_state);
uint8_t twi1_masterBlockingWrite(uint8_t slave_address, uint8_t* write_data, uint8_t bytes_to_write, TWI1_PROTOCALL_t send_stop);

TWI1_RD_t twi1_masterAsyncRead(uint8_t slave_address, uint8_t bytes_to_read, TWI1_PROTOCALL_t send_stop);
TWI1_RD_STAT_t twi1_masterAsyncRead_status(void);
uint8_t twi1_masterAsyncRead_getBytes(uint8_t *read_data);
uint8_t twi1_masterRead(uint8_t slave_address, uint8_t* read_data, uint8_t bytes_to_read, TWI1_PROTOCALL_t send_stop, TWI1_LOOP_STATE_t *loop_state);
uint8_t twi1_masterBlockingRead(uint8_t slave_address, uint8_t* read_data, uint8_t bytes_to_read, TWI1_PROTOCALL_t send_stop);

uint8_t twi1_masterWriteRead(uint8_t slave_address, uint8_t* write_data, uint8_t bytes_to_write, uint8_t* read_data, uint8_t bytes_to_read, TWI1_LOOP_STATE_t *loop_state);

uint8_t twi1_slaveAddress(uint8_t slave);
uint8_t twi1_fillSlaveTxBuffer(const uint8_t* slave_data, uint8_t bytes_to_send);
void twi1_registerSlaveRxCallback( void (*function)(uint8_t*, uint8_t) );
void twi1_registerSlaveTxCallback( void (*function)(void) );

