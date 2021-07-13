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
#include "../lib/adc_bsd.h"
#include "../lib/timers_bsd.h"
#include "../lib/uart1_bsd.h"
#include "../lib/references.h"
#include "analog.h"

static unsigned long serial_print_started_at;

static uint8_t adc_ch;

static uint8_t debug_print_step = 0;

/* print adc intiger values to stram, return false (zero) if an issuse occures*/
uint8_t adc_to_json(FILE *__stream, unsigned long serial_print_delay_ticks)
{
    if ( (debug_print_step == 0) )
    {
        // print in steps otherwise the serial buffer will fill and block the program from running
        serial_print_started_at = tickAtomic();
        fprintf_P(__stream, PSTR("{"));
        adc_ch = ADC_CH_ADC1;
        debug_print_step = 1;
    }
    else if ( (debug_print_step == 1) )
    { // use the channel as an index in the JSON reply
        switch (adc_ch)
        {
            case ADC_CH_ADC1:
            case ADC_CH_ADC2:
            case ADC_CH_ADC3:
            case ADC_CH_ADC4:
                fprintf_P(__stream, PSTR("\"ADC%d\":"),adc_ch);
                break;

            default:
                fprintf_P(__stream, PSTR("\"err\":\"AdcNotChannel\"}\r\n"));
                return 0;
        }
        debug_print_step = 2; 
    }
    else if ( (debug_print_step == 2) )
    {
        int temp_adc = adcAtomic((ADC_CH_t) adc_ch);

        // There are values from 0 to 4095 for 4096 slots where each reperesents 1/4096 of the reference.
        // Slot 4095 also includes higher values e.g., VREF*(4095/4096) and up.
        fprintf_P(__stream, PSTR("\"%d\""), temp_adc);

        if ( (adc_ch+1) >= ADC_CHANNELS) 
        {
            fprintf_P(__stream, PSTR("}\r\n"));
            debug_print_step = 3;
        }
        else
        {
            fprintf_P(__stream, PSTR(","));
            adc_ch++;
            debug_print_step = 1;
        }
    }
    else if ( (debug_print_step == 3) ) 
    { // delay between JSON printing
        unsigned long kRuntime= elapsed(&serial_print_started_at);
        if ((kRuntime) > (serial_print_delay_ticks))
        {
            debug_print_step = 0; /* This keeps looping debug output forever */
        }
    }
    return 1;
}
