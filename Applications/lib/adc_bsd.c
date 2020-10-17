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

volatile int adc[ADC_CHANNELS];
volatile uint8_t adc_channel;
volatile uint8_t ADC_auto_conversion;
volatile VREF_REFSEL_t analog_reference;
volatile uint8_t adc_isr_status;

static uint8_t free_running; // if true loop thru channels continuously

// setup the ADC channel for reading
void channel_setup(void)
{
    switch (adc_channel)
    {
    case 0:
        ADC0.CTRLD = ADC_INITDLY_DLY16_gc; // the reference needs some time to stabalize.
        ADC0.CTRLA = ADC_RESSEL_12BIT_gc | (0<<ADC_CONVMODE_bp); //12-bit resolution, SINGLEENDED
        VREF.ADC0REF = VREF_REFSEL_VDD_gc; // Always On bit (see datasheet) is not selected so after each reading the referance will disconnect
        ADC0.MUXPOS = ADC_MUXPOS_AIN0_gc; // select +ADC side to use AIN0
        //ADC0.MUXNEG = ADC_MUXNEG_GND_gc; // in diff mode select -ADC side to use 0V
        break;
    case 1:
        ADC0.CTRLD = ADC_INITDLY_DLY16_gc; // the reference needs some time to stabalize.
        ADC0.CTRLA = ADC_RESSEL_12BIT_gc | (0<<ADC_CONVMODE_bp); //12-bit resolution, SINGLEENDED
        VREF.ADC0REF = VREF_REFSEL_VDD_gc; // Always On bit (see datasheet) is not selected so after each reading the referance will disconnect
        ADC0.MUXPOS = ADC_MUXPOS_AIN1_gc; // select +ADC side to use AIN1
        //ADC0.MUXNEG = ADC_MUXNEG_GND_gc; // in diff mode select -ADC side to use 0V
        break;
    case 2:
        ADC0.CTRLD = ADC_INITDLY_DLY16_gc; // the reference needs some time to stabalize.
        ADC0.CTRLA = ADC_RESSEL_12BIT_gc | (0<<ADC_CONVMODE_bp); //12-bit resolution, SINGLEENDED
        VREF.ADC0REF = VREF_REFSEL_VDD_gc; // Always On bit (see datasheet) is not selected so after each reading the referance will disconnect
        ADC0.MUXPOS = ADC_MUXPOS_AIN2_gc; // select +ADC side to use AIN2
        //ADC0.MUXNEG = ADC_MUXNEG_GND_gc; // in diff mode select -ADC side to use 0V
        break;
    case 3:
        ADC0.CTRLD = ADC_INITDLY_DLY16_gc; // the reference needs some time to stabalize.
        ADC0.CTRLA = ADC_RESSEL_12BIT_gc | (0<<ADC_CONVMODE_bp); //12-bit resolution, SINGLEENDED
        VREF.ADC0REF = VREF_REFSEL_VDD_gc; // Always On bit (see datasheet) is not selected so after each reading the referance will disconnect
        ADC0.MUXPOS = ADC_MUXPOS_AIN3_gc; // select +ADC side to use AIN3
        //ADC0.MUXNEG = ADC_MUXNEG_GND_gc; // in diff mode select -ADC side to use 0V
        break;
    case 4:
        ADC0.CTRLD = ADC_INITDLY_DLY16_gc; // the reference needs some time to stabalize.
        ADC0.CTRLA = ADC_RESSEL_12BIT_gc | (0<<ADC_CONVMODE_bp); //12-bit resolution, SINGLEENDED
        VREF.ADC0REF = VREF_REFSEL_VDD_gc; // Always On bit (see datasheet) is not selected so after each reading the referance will disconnect
        ADC0.MUXPOS = ADC_MUXPOS_AIN4_gc; // select +ADC side to use AIN4
        //ADC0.MUXNEG = ADC_MUXNEG_GND_gc; // in diff mode select -ADC side to use 0V
        break;
    case 5:
        ADC0.CTRLD = ADC_INITDLY_DLY16_gc; // the reference needs some time to stabalize.
        ADC0.CTRLA = ADC_RESSEL_12BIT_gc | (0<<ADC_CONVMODE_bp); //12-bit resolution, SINGLEENDED
        VREF.ADC0REF = VREF_REFSEL_VDD_gc; // Always On bit (see datasheet) is not selected so after each reading the referance will disconnect
        ADC0.MUXPOS = ADC_MUXPOS_AIN5_gc; // select +ADC side to use AIN5
        //ADC0.MUXNEG = ADC_MUXNEG_GND_gc; // in diff mode select -ADC side to use 0V
        break;
    case 6:
        ADC0.CTRLD = ADC_INITDLY_DLY16_gc; // the reference needs some time to stabalize.
        ADC0.CTRLA = ADC_RESSEL_12BIT_gc | (0<<ADC_CONVMODE_bp); //12-bit resolution, SINGLEENDED
        VREF.ADC0REF = VREF_REFSEL_VDD_gc; // Always On bit (see datasheet) is not selected so after each reading the referance will disconnect
        ADC0.MUXPOS = ADC_MUXPOS_AIN6_gc; // select +ADC side to use AIN6
        //ADC0.MUXNEG = ADC_MUXNEG_GND_gc; // in diff mode select -ADC side to use 0V
        break;
    case 7:
        ADC0.CTRLD = ADC_INITDLY_DLY16_gc; // the reference needs some time to stabalize.
        ADC0.CTRLA = ADC_RESSEL_12BIT_gc | (0<<ADC_CONVMODE_bp); //12-bit resolution, SINGLEENDED
        VREF.ADC0REF = VREF_REFSEL_VDD_gc; // Always On bit (see datasheet) is not selected so after each reading the referance will disconnect
        ADC0.MUXPOS = ADC_MUXPOS_AIN7_gc; // select +ADC side to use AIN7
        //ADC0.MUXNEG = ADC_MUXNEG_GND_gc; // in diff mode select -ADC side to use 0V
        break;
    default:
        adc_channel = 0;
        ADC0.CTRLD = ADC_INITDLY_DLY16_gc; // the reference needs some time to stabalize.
        ADC0.CTRLA = ADC_RESSEL_12BIT_gc | (0<<ADC_CONVMODE_bp); //12-bit resolution, SINGLEENDED
        VREF.ADC0REF = VREF_REFSEL_VDD_gc; // Always On bit (see datasheet) is not selected so after each reading the referance will disconnect
        ADC0.MUXPOS = ADC_MUXPOS_AIN0_gc; // select +ADC side to use AIN0
        //ADC0.MUXNEG = ADC_MUXNEG_GND_gc; // in diff mode select -ADC side to use 0V
        break;
    }
}


// The conversion result is available in ADC0.RES.
ISR(ADC0_RESRDY_vect) 
{
    adc[adc_channel] = ADC0.RES;
    
    ++adc_channel;
    if (adc_channel >= ADC_CHANNELS) 
    {
        adc_channel = 0;
        adc_isr_status = ISR_ADCBURST_DONE; // mark to notify burst is done
    }

    channel_setup();

    // Enable the ADC by writing a ‘1’ to the ADC Enable (ENABLE) bit in the ADCn.CTRLA register.
    if (adc_channel != 0)
    {
        ADC0.CTRLA |= (1<<ADC_ENABLE_bp);
        // ADC0.INTCTRL = (1<<ADC_RESRDY_bp); // ISR's are running
    }
    else if (free_running) // do not confuse with Bit 1 of ADC0.CTRLA which would loop on the same channel
    {
        ADC0.CTRLA |= (1<<ADC_ENABLE_bp);
        adc_isr_status = ISR_ADCBURST_START;
    }
    else
    {
        ADC0.INTCTRL = (0<<ADC_RESRDY_bp); // turn off ISR since channels have been scaned
    }
    
}


// Select a referance (VREF_REFSEL_VDD_gc, VREF_REFSEL_1V024_gc) and initialize ADC but do not start it.
// also used to init for auto conversion
void init_ADC_single_conversion(VREF_REFSEL_t reference)
{
    analog_reference = reference; // save it for use in the ISR
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
    ADC0.SAMPCTRL=14; //16 ADC clock sampling time for slow mode


    // Enable ADC 
    ADC0.CTRLD = ADC_INITDLY_DLY16_gc; // the reference needs time to stabalize.
    ADC0.CTRLA = ADC_RESSEL_12BIT_gc | (0<<ADC_CONVMODE_bp);

    VREF.ADC0REF = reference; // Always On bit (see datasheet) is not selected so after each reading the referance will disconnect

    ADC0.MUXPOS = ADC_MUXPOS_AIN0_gc; // init with +ADC to AIN0
    //ADC0.MUXNEG = ADC_MUXNEG_GND_gc; // init with -ADC to 0V

    ADC_auto_conversion = 0; 
}


// Before setting the ADC scan mode, use init_ADC_single_conversion 
// to select reference and set the adc_clock pre-scaler. This call will start 
// taking readings on each channel the ISR iterates over and holds the result 
// in a buffer.
void enable_ADC_auto_conversion(uint8_t free_run)
{
    adc_channel = 0;
    adc_isr_status = ISR_ADCBURST_START; // mark so we know new readings are arriving
    free_running = free_run;

    // Start the first Conversion (ISR will start each one after the previous is done)
    ADC0.CTRLA |= (1<<ADC_ENABLE_bp);
    ADC0.INTCTRL = (1<<ADC_RESRDY_bp); // turn on ISR's
    ADC_auto_conversion =1;
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

// ADC single channel conversion (blocking)
int adcSingle(uint8_t channel)
{
    adc_channel = channel;
    channel_setup();
    ADC0.CTRLA |= (1<<ADC_ENABLE_bp);
    while ( !(ADC0.INTFLAGS & ADC_RESRDY_bm) );
    int local = ADC0.RES;
    return local;
}
