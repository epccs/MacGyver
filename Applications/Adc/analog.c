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
#include <avr/pgmspace.h>
#include <util/atomic.h>
#include <avr/eeprom.h> 
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include "../lib/parse.h"
#include "../lib/adc_bsd.h"
#include "../lib/rpu_mgr.h"
#include "../lib/twi0_bsd.h"
#include "../lib/timers_bsd.h"
#include "../lib/uart0_bsd.h"
#include "../lib/references.h"
#include "analog.h"

static unsigned long serial_print_started_at;

static uint8_t adc_arg_index;

/* return adc corrected values */
void Analogf(unsigned long serial_print_delay_ticks)
{
    if ( (command_done == 10) )
    {
        // check that arguments are digit in the range 0..7
        for (adc_arg_index=0; adc_arg_index < arg_count; adc_arg_index++) 
        {
            if ( ( !( isdigit(arg[adc_arg_index][0]) ) ) || (atoi(arg[adc_arg_index]) < ADC_CH_ADC0) || (atoi(arg[adc_arg_index]) > (ADC_CHANNELS+ADC_CH_MGR_MAX_NOT_A_CH)) )
            {
                printf_P(PSTR("{\"err\":\"AdcChOutOfRng\"}\r\n"));
                initCommandBuffer();
                return;
            }
        }
        // if references failed to loaded show an error
        if (ref_loaded == VREF_LOADED_ERR)
        {
            printf_P(PSTR("{\"err\":\"AdcRefNotLoaded\"}\r\n"));
            initCommandBuffer();
            return;
        }

        // if calibrations failed to loaded show an error
        if (cal_loaded == CALIBRATE_LOADED_ERR)
        {
            printf_P(PSTR("{\"err\":\"AdcCalNotLoaded\"}\r\n"));
            initCommandBuffer();
            return;
        }

        // print in steps otherwise the serial buffer will fill and block the program from running
        serial_print_started_at = tickAtomic();
        printf_P(PSTR("{"));
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
                printf_P(PSTR("\"ADC%s\":"),arg[adc_arg_index]);
                break;

            default:
                printf_P(PSTR("{\"err\":\"AdcNotChannel\"}\r\n"));
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
        int temp_adc = adcAtomic((ADC_CH_t) arg_indx_channel);
        float *ptr_temp_ref = adcConfMap[arg_indx_channel].ref;
        float temp_ref = *ptr_temp_ref;
        float temp_ch_calibration_value = adcConfMap[arg_indx_channel].calibration;
        float corrected = temp_adc*temp_ref*temp_ch_calibration_value;
        printf_P(PSTR("\"%1.4f=%d*%1.4f*%1.4f\""), corrected,temp_adc,temp_ref,temp_ch_calibration_value);

        if ( (adc_arg_index+1) >= arg_count) 
        {
            printf_P(PSTR("}\r\n"));
            command_done = 21;
        }
        else
        {
            printf_P(PSTR(","));
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
void Analogd(unsigned long serial_print_delay_ticks)
{
    if ( (command_done == 10) )
    {
        // check that arguments are digit in the range 0..7
        for (adc_arg_index=0; adc_arg_index < arg_count; adc_arg_index++) 
        {
            if ( ( !( isdigit(arg[adc_arg_index][0]) ) ) || (atoi(arg[adc_arg_index]) < ADC_CH_ADC0) || (atoi(arg[adc_arg_index]) > (ADC_CHANNELS+ADC_CH_MGR_MAX_NOT_A_CH)) )
            {
                printf_P(PSTR("{\"err\":\"AdcChOutOfRng\"}\r\n"));
                initCommandBuffer();
                return;
            }
        }
        // if references failed to loaded show an error
        if (ref_loaded == VREF_LOADED_ERR)
        {
            printf_P(PSTR("{\"err\":\"AdcRefNotLoaded\"}\r\n"));
            initCommandBuffer();
            return;
        }

        // if calibrations failed to loaded show an error
        if (cal_loaded == CALIBRATE_LOADED_ERR)
        {
            printf_P(PSTR("{\"err\":\"AdcCalNotLoaded\"}\r\n"));
            initCommandBuffer();
            return;
        }

        // print in steps otherwise the serial buffer will fill and block the program from running
        serial_print_started_at = tickAtomic();
        printf_P(PSTR("{"));
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
                printf_P(PSTR("\"ADC%s\":"),arg[adc_arg_index]);
                break;

            default:
                printf_P(PSTR("\"err\":\"AdcNotChannel\"}\r\n"));
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
        int temp_adc = adcSingle((ADC_CH_t) arg_indx_channel);
        printf_P(PSTR("\"%d\""), temp_adc);

        if ( (adc_arg_index+1) >= arg_count) 
        {
            printf_P(PSTR("}\r\n"));
            command_done = 21;
        }
        else
        {
            printf_P(PSTR(","));
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
