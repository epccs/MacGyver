/*
references is a library used to load and set analog conversion references in EEPROM. 
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
#include "adc_bsd.h"
#include "references.h"

VREF_LOADED_t ref_loaded;
CALIBRATE_LOADED_t cal_loaded;
float ref_extern_vdd; // VDD is from the 5V@1A5 SMPS supply (^1 is hacked to input from USB)
float ref_intern_1v0; // 1V024 +/- 4%, but a bandgap refernace is temperature stable
float ref_intern_2v0; // 2V048 +/- 4%, but a bandgap refernace is temperature stable
float ref_intern_4v1; // 4V096 +/- 4%, but a bandgap refernace is temperature stable

struct AdcConf_Map adcConfMap[ADC_CHANNELS]; // Array of ADC config struct. The header has the struct 

// TODO: load referances from manager over I2C
VREF_LOADED_t LoadAnalogRef()
{
    uint16_t i2c_success = 1; // i2c will take several scan loops to finish
    switch (ref_loaded)
    {
    case VREF_LOADED_NO:
        ref_extern_vdd = 5.0;
        if (i2c_success) ref_loaded = VREF_LOADED_VDD;
        break;
    case VREF_LOADED_VDD:
        ref_intern_1v0 = 1.024;
        if (i2c_success) ref_loaded = VREF_LOADED_1V0;
        break;
    case VREF_LOADED_1V0:
        ref_intern_2v0 = 2.048;
        if (i2c_success) ref_loaded = VREF_LOADED_2V0;
        break;
    case VREF_LOADED_2V0:
        ref_intern_4v1 = 4.096;
        if (i2c_success) ref_loaded = VREF_LOADED_4V1;
        break;
    case VREF_LOADED_4V1:
        ref_loaded = VREF_LOADED_DONE;
        break;
    case VREF_LOADED_DONE:
        break;
    default:
        case VREF_LOADED_ERR:
        break;
    }
    return ref_loaded;
}

// Load the hardware setting for ADC channel operation
// TODO: load calibrations from manager over I2C
CALIBRATE_LOADED_t LoadAdcConfig()
{
    uint16_t i2c_success = 1; // i2c will take several scan loops to finish each cal. It is a place holder for the code.
    ADC_CH_t ch; // channels may be differential or internal things (TEMPSENSE, DAC, REF0...)
    switch (cal_loaded)
    {
    case CALIBRATE_LOADED_NO:
        ch = ADC_CH_ADC1;
        adcConfMap[ch].calibration = 1.0/4096;
        adcConfMap[ch].ref = &ref_extern_vdd;
        adcConfMap[ch].adc0ref = VREF_REFSEL_VDD_gc;
        adcConfMap[ch].muxpos = ADC_MUXPOS_AIN0_gc;
        adcConfMap[ch].muxneg = ADC_MUXNEG_GND_gc;
        adcConfMap[ch].sampctrl = 0;
        if (i2c_success) cal_loaded = CALIBRATE_LOADED_CH1;
        break;
    case CALIBRATE_LOADED_CH1:
        ch = ADC_CH_ADC2;
        adcConfMap[ch].calibration = 1.0/4096;
        adcConfMap[ch].ref = &ref_extern_vdd;
        adcConfMap[ch].adc0ref = VREF_REFSEL_VDD_gc;
        adcConfMap[ch].muxpos = ADC_MUXPOS_AIN2_gc;
        adcConfMap[ch].muxneg = ADC_MUXNEG_GND_gc;
        adcConfMap[ch].sampctrl = 0;
        if (i2c_success) cal_loaded = CALIBRATE_LOADED_CH2;
        break;
    case CALIBRATE_LOADED_CH2:
        ch = ADC_CH_ADC3;
        adcConfMap[ch].calibration = 1.0/4096;
        adcConfMap[ch].ref = &ref_extern_vdd;
        adcConfMap[ch].adc0ref = VREF_REFSEL_VDD_gc;
        adcConfMap[ch].muxpos = ADC_MUXPOS_AIN3_gc;
        adcConfMap[ch].muxneg = ADC_MUXNEG_GND_gc;
        adcConfMap[ch].sampctrl = 0;
        if (i2c_success) cal_loaded = CALIBRATE_LOADED_CH3;
        break;
    case CALIBRATE_LOADED_CH3:
        ch = ADC_CH_ADC4;
        adcConfMap[ch].calibration = 1.0/4096;
        adcConfMap[ch].ref = &ref_extern_vdd;
        adcConfMap[ch].adc0ref = VREF_REFSEL_VDD_gc;
        adcConfMap[ch].muxpos = ADC_MUXPOS_AIN4_gc;
        adcConfMap[ch].muxneg = ADC_MUXNEG_GND_gc;
        adcConfMap[ch].sampctrl = 0;
        if (i2c_success) cal_loaded = CALIBRATE_LOADED_CH4;
        break;
    case CALIBRATE_LOADED_CH4:
        cal_loaded = CALIBRATE_LOADED_DONE;
        break;

    default:
        cal_loaded = CALIBRATE_LOADED_ERR;
        break;
    }

    return cal_loaded;
}
