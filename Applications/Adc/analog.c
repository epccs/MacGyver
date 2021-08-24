/*
analog is a library that returns Analog Conversions for channels 
Copyright (C) 2019 Ronald Sutherland

Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES 
WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE 
FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY 
DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, 
WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, 
ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

Note the library files are LGPL, e.g., you need to publish changes of them but can derive from this 
source and copyright or distribute as you see fit (it is Zero Clause BSD).

https://en.wikipedia.org/wiki/BSD_licenses#0-clause_license_(%22Zero_Clause_BSD%22)
*/

#include <stdbool.h>
#include <string.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <avr/eeprom.h> 
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "../lib/parse.h"
#include "../lib/adc_bsd.h"
#include "../lib/rpu_mgr.h"
#include "../lib/twi.h"
#include "../lib/timers_bsd.h"
#include "../lib/uart0_bsd.h"
#include "../lib/references.h"
#include "analog.h"

static unsigned long serial_print_started_at;

static uint8_t adc_arg_index;

#define APP_CB_PORT_ALT_I 0
#define APP_CB_PORT_ALT_V 1
#define APP_CB_PORT_PWR_I 2
#define APP_CB_PORT_PWR_V 3

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

static uint8_t twi0_slave_status_cpy;
#define LAST_OP_A 0
#define LAST_OP_R 1
#define LAST_OP_W 2
static uint8_t twi0_last_op; // last operation e.g., read, write, address

static bool got_twi0_;
static uint8_t *got_twi0_buf = BufferC;
static uint8_t got_twi0BufferLength;
static uint8_t got_twi0BufferIndex;

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

// The manage will update ALT_I, ALT_V, PWR_I, and PWR_V with this
// think of each as a port at an address, and this server handels the clients request
bool twisCallback(twis_irqstate_t state, uint8_t statusReg) {
    bool ret = true;

    switch( state ) {
        case TWIS_ADDRESSED:
            ret = twi0_addr_verified = (twis_lastAddress() == toApp_addr); // test address true to proceed with read or write
            twi0_slave_status_cpy = statusReg;
            if (twi0RxBufferLength) {
                // copy receive buffer into transmit in case next operation is read (so it can echo)
                move_buffer(twi0RxBuffer, &twi0RxBufferLength, twi0TxBuffer, &twi0TxBufferLength, &twi0TxBufferIndex);
            }
            twi0_last_op = LAST_OP_A;
            break;
        case TWIS_MREAD:
            if (twi0TxBufferIndex < twi0TxBufferLength) {
                twis_write( twi0TxBuffer[twi0TxBufferIndex] );
                twi0TxBufferIndex++;
                ret = true; // more data is in the Tx buffer
            }
            // since the master ACK's the read it can keep reading 0xFF all it wants.
            twi0_last_op = LAST_OP_R;
            break;
        case TWIS_MWRITE:
            twi0RxBuffer[twi0RxBufferLength] = twis_read();
            twi0RxBufferLength++;
            ret = (twi0RxBufferLength < BUFF_SIZE); //true to proceed
            twi0_last_op = LAST_OP_W;
            break;
        case TWIS_STOPPED: 
            if (twi0TxBufferLength) { // stop after
                if (twi0RxBufferLength) { // write+write
                    //print_Op2_buf_if_possible(twi0_last_op, twi0RxBuffer, twi0RxBufferLength, twis_lastAddress());
                }
                else { // write+read (echo)
                    //print_Op2_buf_if_possible(twi0_last_op, twi0TxBuffer, twi0TxBufferLength, twis_lastAddress());
                    move_buffer(twi0TxBuffer, &twi0TxBufferLength, got_twi0_buf, &got_twi0BufferLength, &got_twi0BufferIndex); // duplicate echo into got_twi0_buf for use in application
                    got_twi0_ = true;
                }
            } else if (twi0RxBufferLength) { 
                // stop after write
                if (twi0RxBufferLength == 3) { // port + int16_t
                    uint16_t src = 0;
                    src += ((uint16_t)twi0TxBuffer[1])<<8;
                    src += ((uint16_t)twi0TxBuffer[2]);
                    switch (twi0TxBuffer[0])
                    {
                    case APP_CB_PORT_ALT_I:
                        memcpy(&adcMgrConfMap[ADC_CH_MGR_ALT_I].value, &src, sizeof adcMgrConfMap[ADC_CH_MGR_ALT_I].value);
                        break;
                    case APP_CB_PORT_ALT_V:

                        memcpy(&adcMgrConfMap[ADC_CH_MGR_ALT_V].value, &src, sizeof adcMgrConfMap[ADC_CH_MGR_ALT_V].value);
                        break;
                    case APP_CB_PORT_PWR_I:
                        memcpy(&adcMgrConfMap[ADC_CH_MGR_PWR_I].value, &src, sizeof adcMgrConfMap[ADC_CH_MGR_PWR_I].value);
                        break;
                    case APP_CB_PORT_PWR_V:
                        memcpy(&adcMgrConfMap[ADC_CH_MGR_PWR_V].value, &src, sizeof adcMgrConfMap[ADC_CH_MGR_PWR_V].value);
                        break;

                    default:
                        break;
                    }
                }
                
            } else if (twi0_last_op == LAST_OP_A) { 
                // got a ping
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




/* return adc corrected values */
void Analogf(FILE *stream, unsigned long serial_print_delay_ticks)
{
    if ( (command_done == 10) )
    {
        // check that arguments are digit in the range 0..7
        for (adc_arg_index=0; adc_arg_index < arg_count; adc_arg_index++) 
        {
            if ( ( !( isdigit(arg[adc_arg_index][0]) ) ) || (atoi(arg[adc_arg_index]) < ADC_CH_ADC0) || (atoi(arg[adc_arg_index]) > (ADC_CHANNELS + ADC_CHANNELS_MGR)) )
            {
                fprintf_P(stream, PSTR("{\"err\":\"AdcChOutOfRng\"}\r\n"));
                initCommandBuffer();
                return;
            }
        }
        // if references failed to loaded show an error
        if (ref_loaded == VREF_LOADED_ERR)
        {
            fprintf_P(stream, PSTR("{\"err\":\"AdcRefNotLoaded\"}\r\n"));
            initCommandBuffer();
            return;
        }

        // if calibrations failed to loaded show an error
        if (cal_loaded == CALIBRATE_LOADED_ERR)
        {
            fprintf_P(stream, PSTR("{\"err\":\"AdcCalNotLoaded\"}\r\n"));
            initCommandBuffer();
            return;
        }

        // print in steps otherwise the serial buffer will fill and block the program from running
        serial_print_started_at = tickAtomic();
        fprintf_P(stream, PSTR("{"));
        adc_arg_index= 0;
        command_done = 11;
    }
    else if ( (command_done == 11) )
    { // use the channel as an index in the JSON reply
        uint8_t arg_indx_channel = atoi(arg[adc_arg_index]);
        switch (arg_indx_channel)
        {
            case ADC_CH_ADC0:
            case ADC_CH_ADC1:
            case ADC_CH_ADC2:
            case ADC_CH_ADC3:
            case ADC_CH_ADC4:
            case ADC_CH_ADC5:
            case ADC_CH_ADC6:
            case ADC_CH_ADC7:
                fprintf_P(stream, PSTR("\"ADC%s\":"),arg[adc_arg_index]);
                break;
            case (ADC_CHANNELS+ADC_CH_MGR_ALT_I):
                fprintf_P(stream, PSTR("\"ALT_I\":"),arg[adc_arg_index]);
                break;
            case (ADC_CHANNELS+ADC_CH_MGR_ALT_V):
                fprintf_P(stream, PSTR("\"ALT_V\":"),arg[adc_arg_index]);
                break;
            case (ADC_CHANNELS+ADC_CH_MGR_PWR_I):
                fprintf_P(stream, PSTR("\"PWR_I\":"),arg[adc_arg_index]);
                break;
            case (ADC_CHANNELS+ADC_CH_MGR_PWR_V):
                fprintf_P(stream, PSTR("\"PWR_V\":"),arg[adc_arg_index]);
                break;

            default:
                fprintf_P(stream, PSTR("{\"err\":\"AdcNotChannel\"}\r\n"));
                initCommandBuffer();
                return;
        }
        command_done = 20; 
    }
    else if ( (command_done == 20) )
    {
        uint8_t arg_indx_channel = atoi(arg[adc_arg_index]);

        // There are values from 0 to 4095 for 4096 slots where each reperesents 1/4096 of the reference.
        // Slot 4095 also includes higher values e.g., VREF*(4095/4096) and up.
        int temp_adc = 0;
        switch (arg_indx_channel)
        {
            case ADC_CH_ADC0:
            case ADC_CH_ADC1:
            case ADC_CH_ADC2:
            case ADC_CH_ADC3:
            case ADC_CH_ADC4:
            case ADC_CH_ADC5:
            case ADC_CH_ADC6:
            case ADC_CH_ADC7:
                temp_adc = adcAtomic((ADC_CH_t) arg_indx_channel);
                break;
            case (ADC_CHANNELS+ADC_CH_MGR_ALT_I):
                temp_adc = adcMgrConfMap[ADC_CH_MGR_ALT_I].value;
                break;
            case (ADC_CHANNELS+ADC_CH_MGR_ALT_V):
                temp_adc = adcMgrConfMap[ADC_CH_MGR_ALT_V].value;
                break;
            case (ADC_CHANNELS+ADC_CH_MGR_PWR_I):
                temp_adc = adcMgrConfMap[ADC_CH_MGR_PWR_I].value;
                break;
            case (ADC_CHANNELS+ADC_CH_MGR_PWR_V):
                temp_adc = adcMgrConfMap[ADC_CH_MGR_PWR_V].value;
                break;
        }
        float *ptr_temp_ref = adcConfMap[arg_indx_channel].ref;
        float temp_ref = *ptr_temp_ref;
        float temp_ch_calibration_value = adcConfMap[arg_indx_channel].calibration;
        float corrected = temp_adc*temp_ref*temp_ch_calibration_value;
        fprintf_P(stream, PSTR("\"%1.4f\""), corrected);

        if ( (adc_arg_index+1) >= arg_count) 
        {
            fprintf_P(stream, PSTR("}\r\n"));
            command_done = 21;
        }
        else
        {
            fprintf_P(stream, PSTR(","));
            adc_arg_index++;
            command_done = 11;
        }
    }
    else if ( (command_done == 21) ) 
    { // delay between JSON printing
        unsigned long kRuntime= elapsed(&serial_print_started_at);
        if ((kRuntime) > (serial_print_delay_ticks))
        {
            command_done = 10; /* This keeps looping output forever (until a Rx char anyway) */
        }
    }
    else
    {
        initCommandBuffer();
    }
}

/* return adc intiger values */
void Analogd(FILE *stream, unsigned long serial_print_delay_ticks)
{
    if ( (command_done == 10) )
    {
        // check that arguments are digit in the range 0..7
        for (adc_arg_index=0; adc_arg_index < arg_count; adc_arg_index++) 
        {
            if ( ( !( isdigit(arg[adc_arg_index][0]) ) ) || (atoi(arg[adc_arg_index]) < ADC_CH_ADC0) || (atoi(arg[adc_arg_index]) > (ADC_CHANNELS + ADC_CHANNELS_MGR)) )
            {
                fprintf_P(stream, PSTR("{\"err\":\"AdcChOutOfRng\"}\r\n"));
                initCommandBuffer();
                return;
            }
        }
        // if references failed to loaded show an error
        if (ref_loaded == VREF_LOADED_ERR)
        {
            fprintf_P(stream, PSTR("{\"err\":\"AdcRefNotLoaded\"}\r\n"));
            initCommandBuffer();
            return;
        }

        // if calibrations failed to loaded show an error
        if (cal_loaded == CALIBRATE_LOADED_ERR)
        {
            fprintf_P(stream, PSTR("{\"err\":\"AdcCalNotLoaded\"}\r\n"));
            initCommandBuffer();
            return;
        }

        // print in steps otherwise the serial buffer will fill and block the program from running
        serial_print_started_at = tickAtomic();
        fprintf_P(stream, PSTR("{"));
        adc_arg_index= 0;
        command_done = 11;
    }
    else if ( (command_done == 11) )
    { // use the channel as an index in the JSON reply
        uint8_t arg_indx_channel = atoi(arg[adc_arg_index]);
        switch (arg_indx_channel)
        {
            case ADC_CH_ADC0:
            case ADC_CH_ADC1:
            case ADC_CH_ADC2:
            case ADC_CH_ADC3:
            case ADC_CH_ADC4:
            case ADC_CH_ADC5:
            case ADC_CH_ADC6:
            case ADC_CH_ADC7:
                fprintf_P(stream, PSTR("\"ADC%s\":"),arg[adc_arg_index]);
                break;
            case (ADC_CHANNELS+ADC_CH_MGR_ALT_I):
                fprintf_P(stream, PSTR("\"ALT_I\":"),arg[adc_arg_index]);
                i2c_get_mgr_adc(ADC_CH_MGR_ALT_I, APP_CB_PORT_ALT_I); // call back port on which server will receive the requested value
                break;
            case (ADC_CHANNELS+ADC_CH_MGR_ALT_V):
                fprintf_P(stream, PSTR("\"ALT_V\":"),arg[adc_arg_index]);
                i2c_get_mgr_adc(ADC_CH_MGR_ALT_V, APP_CB_PORT_ALT_V); // call back port on which server will receive the requested value
                break;
            case (ADC_CHANNELS+ADC_CH_MGR_PWR_I):
                fprintf_P(stream, PSTR("\"PWR_I\":"),arg[adc_arg_index]);
                i2c_get_mgr_adc(ADC_CH_MGR_PWR_I, APP_CB_PORT_PWR_I); // call back port on which server will receive the requested value
                break;
            case (ADC_CHANNELS+ADC_CH_MGR_PWR_V):
                fprintf_P(stream, PSTR("\"PWR_V\":"),arg[adc_arg_index]);
                i2c_get_mgr_adc(ADC_CH_MGR_PWR_V, APP_CB_PORT_PWR_V); // call back port on which server will receive the requested value
                break;

            default:
                fprintf_P(stream, PSTR("\"err\":\"AdcNotChannel\"}\r\n"));
                initCommandBuffer();
                return;
        }
        command_done = 20; 
    }
    else if ( (command_done == 20) )
    {
        uint8_t arg_indx_channel = atoi(arg[adc_arg_index]);
        int temp_adc = 0;

        switch (arg_indx_channel)
        {
            case ADC_CH_ADC0:
            case ADC_CH_ADC1:
            case ADC_CH_ADC2:
            case ADC_CH_ADC3:
            case ADC_CH_ADC4:
            case ADC_CH_ADC5:
            case ADC_CH_ADC6:
            case ADC_CH_ADC7:
                temp_adc = adcAtomic((ADC_CH_t) arg_indx_channel);
                break;
            case (ADC_CHANNELS+ADC_CH_MGR_ALT_I):
                temp_adc = adcMgrConfMap[ADC_CH_MGR_ALT_I].value;
                break;
            case (ADC_CHANNELS+ADC_CH_MGR_ALT_V):
                temp_adc = adcMgrConfMap[ADC_CH_MGR_ALT_V].value;
                break;
            case (ADC_CHANNELS+ADC_CH_MGR_PWR_I):
                temp_adc = adcMgrConfMap[ADC_CH_MGR_PWR_I].value;
                break;
            case (ADC_CHANNELS+ADC_CH_MGR_PWR_V):
                temp_adc = adcMgrConfMap[ADC_CH_MGR_PWR_V].value;
                break;
        }

        // There are values from 0 to 4095 for 4096 slots where each reperesents 1/4096 of the reference.
        // Slot 4095 also includes higher values e.g., VREF*(4095/4096) and up.
        fprintf_P(stream, PSTR("\"%d\""), temp_adc);

        if ( (adc_arg_index+1) >= arg_count) 
        {
            fprintf_P(stream, PSTR("}\r\n"));
            command_done = 21;
        }
        else
        {
            fprintf_P(stream, PSTR(","));
            adc_arg_index++;
            command_done = 11;
        }
    }
    else if ( (command_done == 21) ) 
    { // delay between JSON printing
        unsigned long kRuntime= elapsed(&serial_print_started_at);
        if ((kRuntime) > (serial_print_delay_ticks))
        {
            command_done = 10; /* This keeps looping output forever (until a Rx char anyway) */
        }
    }
    else
    {
        initCommandBuffer();
    }
}
