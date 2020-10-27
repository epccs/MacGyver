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

*/

#include <util/atomic.h>
#include "adc_bsd.h"
#include "references.h"

volatile int adc[ADC_CHANNELS];
volatile ADC_CH_t adc_channel;
volatile VREF_REFSEL_t analog_reference;
volatile uint8_t adc_isr_status;

static uint8_t free_running; // if true loop thru channels continuously

// setup the ADC channel for reading
void channel_setup(void)
{
    VREF.ADC0REF = calMap[adc_channel].adc0ref; // Always On bit (see datasheet) is not selected so after each reading the referance will disconnect
    ADC0.MUXPOS = calMap[adc_channel].muxpos; // select +ADC side
    ADC0.MUXNEG = calMap[adc_channel].muxneg; // select -ADC side
    ADC0.SAMPCTRL = calMap[adc_channel].sampctrl; // extend the ADC sampling time beyond the default two clocks
    ADC0.CTRLD = ADC_INITDLY_DLY16_gc; // the reference needs some time to stabalize.
    ADC0.CTRLA = ADC_ENABLE_bm             // ADC Enable: enabled
                | ADC_RESSEL_12BIT_gc;     // 12-bit mode
                // | ADC_CONVMODE_bm;     // DIFFERENTIAL mode
    ADC0.COMMAND = ADC_STCONV_bm;   // Start conversion
}


// The conversion result is available in ADC0.RES.
ISR(ADC0_RESRDY_vect) 
{
    adc[adc_channel] = ADC0.RES;

    ++adc_channel;
    if (adc_channel >= ADC_CHANNELS) 
    {
        adc_channel = 0;
    }

    if (adc_channel != 0)
    {
        channel_setup();
        ADC0.INTCTRL = ADC_RESRDY_bm; // ISR needs a manual tickling to trigger again
    }
    else if (free_running) // do not confuse with Bit 1 of ADC0.CTRLA which would loop on the same channel
    {
        channel_setup();
        adc_isr_status = ISR_ADCBURST_START;
        ADC0.INTCTRL = ADC_RESRDY_bm; // ISR needs a manual tickling to trigger again
    }
    else
    {
        adc_isr_status = ISR_ADCBURST_DONE; // mark to notify burst is done
    }
}


// Select a referance (VREF_REFSEL_VDD_gc, VREF_REFSEL_1V024_gc) and initialize ADC but do not start it.
// also used to init for auto conversion
void init_ADC_single_conversion(void)
{
    free_running = 0;

    // datasheet wants ADC clock to be faster than 150 kHz.
#if F_CPU >= 24000000
    ADC0.CTRLC = ADC_PRESC_DIV24_gc; //1 MHz
#elif F_CPU >= 20000000
    ADC0.CTRLC = ADC_PRESC_DIV20_gc; //1 MHz
#elif F_CPU >= 16000000
    ADC0.CTRLC = ADC_PRESC_DIV16_gc; //1 MHz
#elif F_CPU >= 12000000
    ADC0.CTRLC = ADC_PRESC_DIV12_gc; //1 MHz
#elif F_CPU >= 8000000
    ADC0.CTRLC = ADC_PRESC_DIV8_gc;  //1 MHz
#elif F_CPU >= 4000000
    ADC0.CTRLC = ADC_PRESC_DIV4_gc;  //1 MHz
#else  
    ADC0.CTRLC = ADC_PRESC_DIV2_gc; // is the lowest setting
#endif

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
        LoadAnalogCal();
    }
}


// Before setting the ADC scan mode, use init_ADC_single_conversion 
// to select reference and set the adc_clock pre-scaler. This call will start 
// taking readings on each channel the ISR iterates over and holds the result 
// in a buffer.
void enable_ADC_auto_conversion(uint8_t free_run)
{
    adc_channel = 0;
    adc_isr_status = ISR_ADCBURST_START; // mark so we know new readings are wip
    free_running = free_run;

    // Start the first Conversion and touch the interupt bit
    channel_setup();
    ADC0.INTCTRL = ADC_RESRDY_bm; // ISR needs tickling
}

// return two byes from the last ADC update with an atomic transaction to make sure ISR does not change it durring the read
int adcAtomic(ADC_CH_t channel)
{
    int local = 0;
    if (channel < ADC_CHANNELS) 
    {
        // an stomic transaction is done by turning off interrupts
        uint8_t oldSREG = SREG;
        cli();           // clear the global interrupt mask.
        local = adc[channel]; // there are two bytes to copy but nothing can change at the moment
        SREG = oldSREG;  // restore global interrupt if they were enabled
    }
    return local;

}

// Check if the conversion is done
uint8_t ADC0_conversionDone(void)
{
    return (ADC0.INTFLAGS & ADC_RESRDY_bm);
}

// ADC single channel conversion (blocking)
int adcSingle(ADC_CH_t channel)
{
    adc_channel = channel;
    channel_setup();
    while ( !ADC0_conversionDone() );
    int local = ADC0.RES;
    return local;
}
