/*
Manager functions are done over the i2c interface between application controller and manager
Copyright (c) 2017 Ronald S. Sutherland

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

#include <stdio.h>
#include <string.h>
#include "twi0_bsd.h"
#include "rpu_mgr.h"

// 1 .. length to long for buffer 
// 2 .. address send, NACK received 
// 3 .. data send, NACK received 
// 4 .. other twi error (e.g., lost bus arbitration, bus error) 
// 5 .. read does not match length
// 6 .. bad command
// 7 .. prevent sending bad data
uint8_t mgr_twiErrorCode;

// largest I2C transaction with manager so far is six bytes.
#define MAX_CMD_SIZE 8

uint8_t txBuffer_[MAX_CMD_SIZE];
uint8_t bytes_to_write_; // master wrties bytes to slave (you may want to zero the last byte txBuffer array)
uint8_t rxBuffer_[MAX_CMD_SIZE];
uint8_t bytes_to_read_; // master reads bytes from slave (you may want to zero the rxBuffer array)
uint8_t i2c_address_; // master address this slave

// command 0 is used to read the address from manager
#define ADDRESS_CMD {0x00,0x00}
#define ADDRESS_CMD_SIZE 2

// command 4 reads the shutdown detected status
#define SHUTDOWN_DETECT_CMD {0x04,0xFF}
#define SHUTDOWN_DETECT_CMD_SIZE 2

// command 5 has the manager pull down the shutdown 
// pin for a time so the host can see it and hault
#define SHUTDOWN_CMD {0x05,0x01}
#define SHUTDOWN_CMD_SIZE 2

// command 6 reads the status bits
#define STATUS_READ_CMD {0x06,0xFF}
#define STATUS_READ_CMD_SIZE 2

// commands 21, 22 have the manger set and report an int.
// The morning and evening threshold 
#define INT_CMD {0x15,0x00,0x00}
#define INT_CMD_SIZE 3

// commands 5 
// have the manger access a read/write array of int. 
// shutdown_halt_curr_limit
#define INT_RW_ARRY_CMD_SIZE 4

// set daynight callback address 49 with cmd 19 on routes 1, 2, 3 (always poke)
// e.g., reports daynight_state on route 1, day evnt on route 2, night evnt on route 3.
#define DAYNIGHT_CALLBK_CMD {0x13,0x31,0x1,0x2,0x3}
#define DAYNIGHT_CALLBK_CMD_SIZE 5

// set battery manager callback address 49 with cmd 16 on route 4 with poke
#define BATTERY_CALLBK_CMD {0x10,0x31,0x4,0x2}
#define BATTERY_CALLBK_CMD_SIZE 4

// set host shutdown callback address 49 with cmd 4 on route 5 with poke
#define HOSTSHUTDOWN_CALLBK_CMD {0x4,0x31,0x5,0x2}
#define HOSTSHUTDOWN_CALLBK_CMD_SIZE 4

// commands 32 will have the manger do an 
// analogRead and pass that to the application
#define ANALOG_RD_CMD {0x20,0x00,0x00}
#define ANALOG_RD_CMD_SIZE 3

// commands 52, 53 and 54  have the manger set and report an
// unsigned long. The evening|morning|millis_now debouce 
// out of range values are ignored
#define ULONGINT_CMD_SIZE 5

// commands 6 
// have the manger access a read/write array of unsigned long. 
// shutdown_[halt_ttl_limit,delay_limit,wearleveling_limit]
#define UL_RW_ARRY_CMD_SIZE 6

// command 38 can have the manger set and report an
// float. Cmd 38 is followed by a select byte, use 0 for EXTERNAL_AVCC and 
// referance 1 for INTERNAL_1V1. Bit 7 of the select byte will save the 
// sent value. e.g., 0x26 0x81 will save the next four bytes to INTERNAL_1V1
#define FLOAT_CMD_SLCT {0x26,0x00,0x00,0x00,0x00,0x00}
#define FLOAT_CMD_SLCT_SIZE 6

#define RPU_BUS_MSTR_CMD_SZ 2
#define I2C_ADDR_OF_BUS_MGR 0x29

// cycle the twi state machine on both the master and slave(s)
void i2c_ping(void)
{ 
    // ping I2C for an RPU bus manager 
    uint8_t i2c_address = I2C_ADDR_OF_BUS_MGR;
    uint8_t data = 0;
    uint8_t length = 0;
    for (uint8_t i =0;1; i++) // try a few times, it is slower starting after power up.
    {
        mgr_twiErrorCode = twi0_masterBlockingWrite(i2c_address, &data, length, TWI0_PROTOCALL_STOP); 
        if (mgr_twiErrorCode == 0) break; // error free code
        if (i>5) return; // give up after 5 trys
    }
    return; 
}

// The manager can pull down the shutdown pin (just like the manual switch) 
// that the Raspberry Pi monitors for halting its operating system.
uint8_t i2c_set_Rpu_shutdown(void)
{ 
    uint8_t i2c_address = I2C_ADDR_OF_BUS_MGR;
    
    // Send the host shutdown command to manager, this should cause 
    // the manager to pull down its pin used to signal host to shutdown
    uint8_t txBuffer[SHUTDOWN_CMD_SIZE] = SHUTDOWN_CMD; 
    uint8_t length = SHUTDOWN_CMD_SIZE;
    mgr_twiErrorCode = twi0_masterBlockingWrite(i2c_address, txBuffer, length, TWI0_PROTOCALL_REPEATEDSTART); 
    if (mgr_twiErrorCode)
    {
        return 0;
    }
    
    // above writes data to manager, this reads data from manager which sends back the same length that was sent
    uint8_t rxBuffer[SHUTDOWN_CMD_SIZE];
    uint8_t bytes_read = twi0_masterBlockingRead(i2c_address, rxBuffer, length, TWI0_PROTOCALL_STOP);
    if ( bytes_read != length )
    {
        mgr_twiErrorCode = 5;
        return 0;
    }
    else
    {
        if ( rxBuffer[1] == txBuffer[1] )
        {
            return 1; // all seems good
        }
        else 
        {
            return 0;
        }
    }
}

// The manager keeps track of when the shutdown pin (manual switch or 
// i2c/SMBus command) is pulled low. Reading will clear the record.
uint8_t i2c_detect_Rpu_shutdown(void)
{ 
    uint8_t i2c_address = I2C_ADDR_OF_BUS_MGR;

    // Send the host shutdown detect command to manager, the manager will return the 
    // shutdown_detected value and then clear it.
    // note: the host can request shutdown via SMBus and this will retain clues for the application.
    uint8_t txBuffer[SHUTDOWN_DETECT_CMD_SIZE] = SHUTDOWN_DETECT_CMD; //detect host shutdown comand 0x04, data place holder 0xFF;
    uint8_t length = SHUTDOWN_DETECT_CMD_SIZE;
    mgr_twiErrorCode = twi0_masterBlockingWrite(i2c_address, txBuffer, length, TWI0_PROTOCALL_REPEATEDSTART); 
    if (mgr_twiErrorCode)
    {
        return 0; // failed
    }
    
    // above writes data to slave, this reads data from slave
    uint8_t rxBuffer[SHUTDOWN_DETECT_CMD_SIZE];
    uint8_t bytes_read = twi0_masterBlockingRead(i2c_address, rxBuffer, length, TWI0_PROTOCALL_STOP);
    if ( bytes_read != length )
    {
        mgr_twiErrorCode = 5;
        return 0;
    }
    else
    {
        return rxBuffer[1];
    }
}

// The manager has the mulitdorp serial address. When I read it 
// over I2C the manage will boradcast a command on its out of band
// channel that places all devices in normal mode (e.g., not p2p or bootload) 
char i2c_get_Rpu_address(void)
{ 
    i2c_ping();
    if ( mgr_twiErrorCode ) return 0;
    uint8_t i2c_address = I2C_ADDR_OF_BUS_MGR;    
    uint8_t txBuffer[ADDRESS_CMD_SIZE] = ADDRESS_CMD;
    uint8_t length = ADDRESS_CMD_SIZE;
    mgr_twiErrorCode = twi0_masterBlockingWrite(i2c_address, txBuffer, length, TWI0_PROTOCALL_REPEATEDSTART); 
    if (mgr_twiErrorCode)
    {
        return 0; // failed
    }

    uint8_t rxBuffer[ADDRESS_CMD_SIZE];
    uint8_t bytes_read = twi0_masterBlockingRead(i2c_address, rxBuffer, length, TWI0_PROTOCALL_STOP);
    if ( bytes_read != length )
    {
        mgr_twiErrorCode = 5;
        return 0;
    }
    else
    {
        return (char)(rxBuffer[1]);
    }
}

// I2C command 32 takes a channel and returns adc[channel]
// channels are ALT_I | ALT_V | PWR_I | PWR_V
int i2c_get_adc_from_manager(uint8_t channel, TWI0_LOOP_STATE_t *loop_state)
{
    // use the int access cmd
    return i2c_int_access_cmd(0x20, (int)channel, loop_state);
}

// The manager has a status byte that has the following bits. 
// 0 .. DTR readback timeout
// 1 .. twi fail
// 2 .. DTR readback not match
// 3 .. host lockout
// 4 .. alternate power enable (ALT_EN) 
// 5 .. SBC power enable (PIPWR_EN) 
// 6 .. daynight fail
uint8_t i2c_read_status(void)
{ 
    uint8_t i2c_address = I2C_ADDR_OF_BUS_MGR;

    uint8_t txBuffer[STATUS_READ_CMD_SIZE] = STATUS_READ_CMD; //detect host shutdown comand 0x04, data place holder 0xFF;
    uint8_t length = STATUS_READ_CMD_SIZE;
    mgr_twiErrorCode = twi0_masterBlockingWrite(i2c_address, txBuffer, length, TWI0_PROTOCALL_REPEATEDSTART); 
    if (mgr_twiErrorCode)
    {
        return 0x02; // set twi fail bit, even though it is not from manager
    }
    
    // above writes data to slave, this reads data from slave
    uint8_t rxBuffer[STATUS_READ_CMD_SIZE];
    uint8_t bytes_read = twi0_masterBlockingRead(i2c_address, rxBuffer, length, TWI0_PROTOCALL_STOP);
    if ( bytes_read != length )
    {
        mgr_twiErrorCode = 5;
        return 0x02;
    }
    else
    {
        return rxBuffer[1]; // return manager status
    }
}

// setup callbacks and poke manager to get daynight_state, day_event, night_event.
// 19 .. cmd plus four bytes 
//       byte 1 is the slave address for manager to send envents
//       byte 2 is route to receive daynight_state changes
//       byte 3 is route to receive day work event
//       byte 4 is route to receive night work event 
void i2c_daynight_cmd(uint8_t dn_callback_addr, uint8_t dn_callback_route, uint8_t d_callback_route, uint8_t n_callback_route)
{ 
    uint8_t i2c_address = I2C_ADDR_OF_BUS_MGR;
    uint8_t txBuffer[DAYNIGHT_CALLBK_CMD_SIZE] = DAYNIGHT_CALLBK_CMD;
    txBuffer[1] = dn_callback_addr;
    txBuffer[2] = dn_callback_route;
    txBuffer[3] = d_callback_route;
    txBuffer[4] = n_callback_route;
    uint8_t length = DAYNIGHT_CALLBK_CMD_SIZE;
    mgr_twiErrorCode = twi0_masterBlockingWrite(i2c_address, txBuffer, length, TWI0_PROTOCALL_REPEATEDSTART); 
    if (mgr_twiErrorCode)
    {
        return; // failed
    }
    
    // above writes data to slave, this reads data from slave
    uint8_t rxBuffer[DAYNIGHT_CALLBK_CMD_SIZE];
    uint8_t bytes_read = twi0_masterBlockingRead(i2c_address, rxBuffer, length, TWI0_PROTOCALL_STOP);
    if ( bytes_read != length )
    {
        mgr_twiErrorCode = 5;
        return;
    }
}

// setup callback and poke the manager to get bm_state (battery manager).
// 16 .. cmd plus two bytes 
//       byte 1 is the slave address for manager to send envents
//       byte 2 is route to receive bm_state changes
//       byte 3 is battery manager enable[1], disable[0], poke[2..254].
void i2c_battery_cmd(uint8_t bm_callback_addr, uint8_t bm_callback_route, uint8_t bm_enable)
{ 
    uint8_t i2c_address = I2C_ADDR_OF_BUS_MGR;
    uint8_t txBuffer[BATTERY_CALLBK_CMD_SIZE] = BATTERY_CALLBK_CMD;
    uint8_t length = BATTERY_CALLBK_CMD_SIZE;
    txBuffer[1] = bm_callback_addr;
    txBuffer[2] = bm_callback_route;
    txBuffer[3] = bm_enable; // enable the state machine
    mgr_twiErrorCode = twi0_masterBlockingWrite(i2c_address, txBuffer, length, TWI0_PROTOCALL_REPEATEDSTART); 
    if (mgr_twiErrorCode)
    {
        return; // failed
    }
    
    // above writes data to slave, this reads data from slave
    uint8_t rxBuffer[BATTERY_CALLBK_CMD_SIZE];
    uint8_t bytes_read = twi0_masterBlockingRead(i2c_address, rxBuffer, length, TWI0_PROTOCALL_STOP);
    if ( bytes_read != length )
    {
        mgr_twiErrorCode = 5;
        return;
    }
}

// setup callback and poke the manager to get hs_state (host shutdown).
// 4 .. cmd plus two bytes 
//      byte 1 is the slave address for manager to send envents
//      byte 2 is route to receive hs_state changes
//      byte 3 is cntl and will bring host UP[1], take host DOWN[0], poke[2..254].
// the manager may not like poke, but is how to find the hs_state without changing it.
void i2c_shutdown_cmd(uint8_t hs_callback_addr, uint8_t hs_callback_route, uint8_t hs_cntl)
{ 
    uint8_t i2c_address = I2C_ADDR_OF_BUS_MGR;
    uint8_t txBuffer[HOSTSHUTDOWN_CALLBK_CMD_SIZE] = HOSTSHUTDOWN_CALLBK_CMD;
    uint8_t length = HOSTSHUTDOWN_CALLBK_CMD_SIZE;
    txBuffer[1] = hs_callback_addr; // a slave callback address of zero will shutdown host, and end callbacks
    txBuffer[2] = hs_callback_route; // CB_ROUTE_HS_STATE
    txBuffer[3] = hs_cntl;
    mgr_twiErrorCode = twi0_masterBlockingWrite(i2c_address, txBuffer, length, TWI0_PROTOCALL_REPEATEDSTART); 
    if (mgr_twiErrorCode)
    {
        return; // failed
    }
    
    // above writes data to slave, this reads data from slave
    uint8_t rxBuffer[HOSTSHUTDOWN_CALLBK_CMD_SIZE];
    uint8_t bytes_read = twi0_masterBlockingRead(i2c_address, rxBuffer, length, TWI0_PROTOCALL_STOP);
    if ( bytes_read != length )
    {
        mgr_twiErrorCode = 5;
        return;
    }
}

// management commands that take r/w+offset byte and use it to access an array of unsigned long prameters's e.g.,
//  6 .. SHUTDOWN_UL_CMD and SHUTDOWN_[TTL_OFFSET|DELAY_OFFSET|WEARLEVEL_OFFSET|...WEARLVL_DONE_AT]
// 18 .. BATTERY_UL_CMD and BATTERY_CHARGE_PWM 
// 21 .. DAYNIGHT_UL_CMD and DAYNIGHT_[MORNING_DEBOUNCE|EVENING_DEBOUNCE|...PWR_MTI_DAY]
unsigned long i2c_ul_rwoff_access_cmd(uint8_t command, uint8_t rw_offset, unsigned long update_with, TWI0_LOOP_STATE_t *loop_state)
{
    if ( !((command == SHUTDOWN_UL_CMD) | (command == BATTERY_UL_CMD) | (command == DAYNIGHT_UL_CMD)) ) 
    {
        mgr_twiErrorCode = 6;
        *loop_state = TWI0_LOOP_STATE_DONE;
        return 0;
    }

    unsigned long value = 0;
    if (*loop_state == TWI0_LOOP_STATE_INIT)
    {
        i2c_address_ = I2C_ADDR_OF_BUS_MGR; //0x29
        bytes_to_write_ = UL_RW_ARRY_CMD_SIZE;
        txBuffer_[0] = command; // replace the command byte
        txBuffer_[1] = rw_offset; // replace the read/write+offset byte
        txBuffer_[2] = (uint8_t)((update_with & 0xFF000000UL)>>24);
        txBuffer_[3] = (uint8_t)((update_with & 0xFF0000UL)>>16);
        txBuffer_[4] = (uint8_t)((update_with & 0xFF00UL)>>8);
        txBuffer_[5] = (uint8_t)(update_with & 0xFFUL);
        txBuffer_[6] = 0;
        bytes_to_read_ = UL_RW_ARRY_CMD_SIZE;
        rxBuffer_[0] = 0;
        rxBuffer_[1] = 0;
        rxBuffer_[2] = 0;
        rxBuffer_[3] = 0;
        rxBuffer_[4] = 0;
        rxBuffer_[5] = 0;
        rxBuffer_[6] = 0;
        *loop_state = TWI0_LOOP_STATE_ASYNC_WRT; // set write state
    }
    else 
    {
        uint8_t bytes_read = twi0_masterWriteRead(i2c_address_, txBuffer_, bytes_to_write_, rxBuffer_, bytes_to_read_, loop_state);
        if( (*loop_state == TWI0_LOOP_STATE_DONE) )
        {
            // twi0_masterWriteRead error code is in bits 5..7
            if(bytes_read & 0xE0)
            {
                mgr_twiErrorCode = twi0_masterAsyncWrite_status(); // bytes_read>>5
                value = 0; // UL does not have NaN
            }
            else
            {
                value = ((unsigned long)(rxBuffer_[2]))<<24;
                value += ((unsigned long)(rxBuffer_[3]))<<16;
                value += ((unsigned long)(rxBuffer_[4]))<<8;
                value +=  (unsigned long)rxBuffer_[5];
            }
        }
    }
    return value;
}

// management commands that take an int to update and return an int e.g. 
// 32 .. takes a ADC_CH_MGR_enum and returns the 10 bit adc reading (ALT_I | ALT_V | PWR_I | PWR_V)
int i2c_int_access_cmd(uint8_t command, int update_with, TWI0_LOOP_STATE_t *loop_state)
{
    if ( (command != 32) ) 
    {
        mgr_twiErrorCode = 6;
        *loop_state = TWI0_LOOP_STATE_DONE;
        return 0;
    }
    if ( (command == 32) && (update_with >= ADC_CH_MGR_MAX_NOT_A_CH) )
    {
        mgr_twiErrorCode = 7;
        *loop_state = TWI0_LOOP_STATE_DONE;
        return -1;
    }

    int value = 0;
    if (*loop_state == TWI0_LOOP_STATE_INIT)
    {
        i2c_address_ = I2C_ADDR_OF_BUS_MGR; //0x29
        bytes_to_write_ = INT_CMD_SIZE;
        txBuffer_[0] = command; // replace the command byte
        txBuffer_[1] = (uint8_t)((update_with & 0xFF00)>>8);
        txBuffer_[2] = (uint8_t)(update_with & 0xFF);
        txBuffer_[3] = 0;
        bytes_to_read_ = INT_CMD_SIZE;
        rxBuffer_[0] = 0;
        rxBuffer_[1] = 0;
        rxBuffer_[2] = 0;
        rxBuffer_[3] = 0;
        *loop_state = TWI0_LOOP_STATE_ASYNC_WRT; // set write state
    }
    else 
    {
        uint8_t bytes_read = twi0_masterWriteRead(i2c_address_, txBuffer_, bytes_to_write_, rxBuffer_, bytes_to_read_, loop_state);
        if( (*loop_state == TWI0_LOOP_STATE_DONE) )
        {
            // twi0_masterWriteRead error code is in bits 5..7
            if(bytes_read & 0xE0)
            {
                mgr_twiErrorCode = twi0_masterAsyncWrite_status(); //bytes_read>>5
                value = 0; // int does not have NaN
            }
            else
            {
                value = ((int)(rxBuffer_[1]))<<8;
                value +=  (int)rxBuffer_[2];
            }
        }
    }
    return value;
}


// management commands that take r/w+offset byte and use it to access an array of int's  e.g.,
//  5 .. SHUTDOWN_INT_CMD and SHUTDOWN_HALT_CURR_OFFSET
// 17 .. BATTERY_INT_CMD and BATTERY_[HIGH|LOW|HOST]
// 20 .. DAYNIGHT_INT_CMD and DAYNIGHT_[MORNING_THRESHOLD|EVENING_THRESHOLD]

int i2c_int_rwoff_access_cmd(uint8_t command, uint8_t rw_offset, int update_with, TWI0_LOOP_STATE_t *loop_state)
{
    if ( !((command == SHUTDOWN_INT_CMD) | (command == BATTERY_INT_CMD) | (command == DAYNIGHT_INT_CMD)) ) 
    {
        mgr_twiErrorCode = 6;
        *loop_state = TWI0_LOOP_STATE_DONE;
        return 0;
    }

    int value = 0;
    if (*loop_state == TWI0_LOOP_STATE_INIT)
    {
        i2c_address_ = I2C_ADDR_OF_BUS_MGR; //0x29
        bytes_to_write_ = INT_RW_ARRY_CMD_SIZE;
        txBuffer_[0] = command; // replace the command byte
        txBuffer_[1] = rw_offset; // replace the read/write+offset byte
        txBuffer_[2] = (uint8_t)((update_with & 0xFF00)>>8);
        txBuffer_[3] = (uint8_t)(update_with & 0xFF);
        txBuffer_[4] = 0;
        bytes_to_read_ = INT_RW_ARRY_CMD_SIZE;
        rxBuffer_[0] = 0;
        rxBuffer_[1] = 0;
        rxBuffer_[2] = 0;
        rxBuffer_[3] = 0;
        rxBuffer_[4] = 0;
        *loop_state = TWI0_LOOP_STATE_ASYNC_WRT; // set write state
    }
    else 
    {
        twi0_masterWriteRead(i2c_address_, txBuffer_, bytes_to_write_, rxBuffer_, bytes_to_read_, loop_state);
        if( (*loop_state == TWI0_LOOP_STATE_DONE) )
        {
            mgr_twiErrorCode = twi0_masterAsyncWrite_status();
            if(!mgr_twiErrorCode)
            {
                value = ((int)(rxBuffer_[2]))<<8;
                value +=  (int)rxBuffer_[3];
            }
        }
    }
    return value;
}


// management commands that select a float to update or return e.g. 
// 38 .. access analog referance
float i2c_float_access_cmd(uint8_t command, uint8_t select, float *update_with, TWI0_LOOP_STATE_t *loop_state)
{
    if ( !( (command == 38) || (command == 33) ) ) 
    {
        mgr_twiErrorCode = 6;
        *loop_state = TWI0_LOOP_STATE_DONE;
        return 0;
    }

    // select=0 EXTERNAL_AVCC, and select=1 for INTERNAL_1V1, bit 7 is used to update eeprom
    if ( (command == 38) && ( (select & 0x7F) >= 2) )  
    {
        mgr_twiErrorCode = 7;
        *loop_state = TWI0_LOOP_STATE_DONE;
        return 0;
    }

    // select = channel 0..3, bit 7 is used to update eeprom
    if ( (command == 33) && ( (select & 0x7F) >= 4) )  
    {
        mgr_twiErrorCode = 7;
        *loop_state = TWI0_LOOP_STATE_DONE;
        return 0;
    }

    float value = 0;
    if (*loop_state == TWI0_LOOP_STATE_INIT)
    {
        i2c_address_ = I2C_ADDR_OF_BUS_MGR; //0x29
        bytes_to_write_ = FLOAT_CMD_SLCT_SIZE;
        txBuffer_[0] = command; // replace the command byte
        txBuffer_[1] = select; // replace the select byte
        uint32_t f_in_u32;
        memcpy(&f_in_u32, update_with, sizeof f_in_u32); // copy float bytes into u32
        txBuffer_[2] = (uint8_t)((f_in_u32 & 0xFF000000UL)>>24);
        txBuffer_[3] = (uint8_t)((f_in_u32 & 0xFF0000UL)>>16);
        txBuffer_[4] = (uint8_t)((f_in_u32 & 0xFF00UL)>>8);
        txBuffer_[5] = (uint8_t)(f_in_u32 & 0xFFUL);
        txBuffer_[6] = 0;
        bytes_to_read_ = FLOAT_CMD_SLCT_SIZE;
        rxBuffer_[0] = 0;
        rxBuffer_[1] = 0;
        rxBuffer_[2] = 0;
        rxBuffer_[3] = 0;
        rxBuffer_[4] = 0;
        rxBuffer_[5] = 0;
        rxBuffer_[6] = 0;
        *loop_state = TWI0_LOOP_STATE_ASYNC_WRT; // set write state
    }
    else
    {
        uint8_t bytes_read = twi0_masterWriteRead(i2c_address_, txBuffer_, bytes_to_write_, rxBuffer_, bytes_to_read_, loop_state);
        if( (*loop_state == TWI0_LOOP_STATE_DONE) )
        {
            // twi0_masterWriteRead error code is in bits 5..7
            if(bytes_read & 0xE0)
            {
                mgr_twiErrorCode = twi0_masterAsyncWrite_status(); // bytes_read>>5
                value = (float)0xFFFFFFFFUL; // return NaN
            }
            else
            {
                uint32_t f_in_u32;
                f_in_u32 = ((uint32_t)(rxBuffer_[2]))<<24;
                f_in_u32 += ((uint32_t)(rxBuffer_[3]))<<16;
                f_in_u32 += ((uint32_t)(rxBuffer_[4]))<<8;
                f_in_u32 +=  (uint32_t)rxBuffer_[5];
                memcpy(&value, &f_in_u32, sizeof value);
            }
        }
    }
    return value;
}