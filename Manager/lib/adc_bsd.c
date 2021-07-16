/*
Interrupt-Driven Analog-to-Digital Converter
Copyright (C) 2020 Ronald Sutherland

Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES 
WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE 
FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY 
DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, 
WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, 
ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

https://en.wikipedia.org/wiki/BSD_licenses#0-clause_license_(%22Zero_Clause_BSD%22)

note: free_running seems like a bad idea now so I have removed it.

*/

#include <util/atomic.h>
#include "timers_bsd.h"
#include "adc_bsd.h"
#include "references.h"

volatile int adc[ADC_CHANNELS];
volatile ADC_CH_t adc_channel;
volatile VREF_REFSEL_t analog_reference;
volatile uint8_t adc_isr_status;

uint8_t adc_auto_conversion;

// setup the ADC channel for reading
void channel_setup(ADC_CH_t ch)
{
    ADC0.COMMAND = ADC_SPCONV_bm;                 // Stop ADC conversion to get a clean value
    ADC0.CTRLA  = 0;                              // disabled
    adc_channel = ch;
    VREF.ADC0REF = adcConfMap[ch].adc0ref;        // after each reading the referance will disconnect
    ADC0.CTRLA = ADC_RESSEL_12BIT_gc;             // 12-bit mode
    //ADC0.CTRLA |= ADC_CONVMODE_bm;                // DIFFERENTIAL mode
#if F_CPU >= 24000000
    ADC0.CTRLC = ADC_PRESC_DIV24_gc;              // 1 MHz DS datasheet ADC clock to be faster than 150 kHz.
#elif F_CPU >= 20000000
    ADC0.CTRLC = ADC_PRESC_DIV20_gc;              // 1 MHz
#elif F_CPU >= 16000000
    ADC0.CTRLC = ADC_PRESC_DIV16_gc;              // 1 MHz
#elif F_CPU >= 12000000
    ADC0.CTRLC = ADC_PRESC_DIV12_gc;              // 1 MHz
#elif F_CPU >= 8000000
    ADC0.CTRLC = ADC_PRESC_DIV8_gc;               // 1 MHz
#elif F_CPU >= 4000000
    ADC0.CTRLC = ADC_PRESC_DIV4_gc;               // 1 MHz
#else  
    ADC0.CTRLC = ADC_PRESC_DIV2_gc;              // the lowest setting
#endif
    ADC0.MUXPOS = adcConfMap[ch].muxpos;          // select +ADC side
    ADC0.MUXNEG = adcConfMap[ch].muxneg;          // select -ADC side
    ADC0.SAMPCTRL = adcConfMap[ch].sampctrl;      // extend the ADC sampling time beyond the default two clocks
    ADC0.CTRLD = ADC_INITDLY_DLY16_gc;            // the reference may need some time to stabalize.
    ADC0.CTRLA |= ADC_ENABLE_bm;                  // ADC Enabled
}

// Start ADC conversion
void start_adc_conversion(void)
{
    ADC0.COMMAND = ADC_STCONV_bm;                 // Start ADC conversion
}

// The conversion result is available in ADC0.RES.
ISR(ADC0_RESRDY_vect) 
{
    adc[adc_channel] = ADC0.RES;        // Clear the interrupt flag by reading the result

    if (adc_channel >= ADC_CH_ADC4)
    {
        adc_isr_status = ISR_ADCBURST_DONE; // mark to notify that burst is done
        adc_auto_conversion = 0;
    }
    else // start next channel
    {
        ADC_CH_t next_ch = adc_channel + 1;
        channel_setup(next_ch);
        start_adc_conversion();
    }
}

    //============
    // public:
    //============

// Select a referance (VREF_REFSEL_VDD_gc, VREF_REFSEL_1V024_gc) and initialize ADC but do not start it.
// also used to init for auto conversion
void init_ADC_single_conversion(void)
{

    // load references or set error
    ref_loaded = VREF_LOADED_NO;
    while(ref_loaded < VREF_LOADED_DONE)
    {
        LoadAnalogRef();
    }

    // load calibrations or set error
    cal_loaded = CALIBRATE_LOADED_NO;
    while(cal_loaded < CALIBRATE_LOADED_DONE)
    {
        LoadAdcConfig();
    }
}

// return two byes from the last ADC update with an atomic transaction to make sure ISR does not change it durring the read
int adcAtomic(ADC_CH_t channel)
{
    int ret = -1;
    if (channel < ADC_CHANNELS) 
    {
        // an stomic transaction is done by turning off interrupts
        uint8_t oldSREG = SREG;
        cli();           // turn off interupts if they are on.
        ret = adc[channel]; // there are two bytes to copy but nothing can change since interupts are off
        SREG = oldSREG;  // restore interrupts if they were enabled
    }
    return ret;

}

// single channel conversion (blocking)
int adcSingle(ADC_CH_t channel)
{
    if ( (adc_auto_conversion) || (channel >= ADC_CHANNELS))
    {
        return -1; // skip conversion and report error when ISR is running since it will corrupt the values
    }
    else
    {
        channel_setup(channel);
        start_adc_conversion();
        while ( !(ADC0.INTFLAGS & ADC_RESRDY_bm) );   // Check if the conversion is done
        int local = ADC0.RES;                         // Clears the interrupt flag
        return local;
    }
}

// Before burst do init_ADC_single_conversion 
// This call will start taking readings on each channel.
// The ISR will iterate through the channels and save the results in a buffer.
void adc_burst(unsigned long *adc_started_at, unsigned long *adc_delay_milsec)
{
    unsigned long kRuntime= elapsed(adc_started_at);
    if ((kRuntime) > (*adc_delay_milsec))
    {
        adc_isr_status = ISR_ADCBURST_START; // mark so we know new readings are wip
        adc_auto_conversion = 1;

        // setup first channel and start conversion
        channel_setup(ADC_CH_ADC1);
        start_adc_conversion();

        // save time for next burst
        *adc_started_at += *adc_delay_milsec; 
    } 
}