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
#include "../lib/parse.h"
#include "adc_bsd.h"
#include "references.h"

VREF_LOADED_t ref_loaded;
CALIBRATE_LOADED_t cal_loaded;
CAL_MGR_LOADED_t cal_mgr_loaded;
float ref_extern_vdd; // VDD is from the 5V@1A5 SMPS supply (^1 is hacked to input from USB)
float ref_intern_1v0; // 1V024 +/- 4%, but a bandgap refernace is temperature stable
float ref_intern_2v0; // 2V048 +/- 4%, but a bandgap refernace is temperature stable
float ref_intern_4v1; // 4V096 +/- 4%, but a bandgap refernace is temperature stable

struct AdcConf_Map adcConfMap[ADC_CHANNELS]; // Array of ADC config struct. The header has the struct

struct AdcMgrConf_Map adcMgrConfMap[ADC_CHANNELS_MGR]; // Array of ADC config struct (alt_i, alt_v, pwr_i, pwr_v) shadow value from manager

// Load the hardware setting for ADC channel operation
// this takes a lot of i2c transactions so it needs to be called over and over until true.
// might want to spin with a group of other things during setup 
// TODO: load calibrations from manager over I2C
bool LoadAdcConfig()
{
    if(ref_loaded < VREF_LOADED_DONE) {
        uint16_t i2c_success = 1; // i2c will take several scan loops to finish
        switch (ref_loaded) {
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
        return false;
    }
    if (cal_loaded < CALIBRATE_LOADED_DONE) {
        uint16_t i2c_success = 1; // i2c will take several scan loops to finish each cal. It is a place holder for the code.
        ADC_CH_t ch; // channels may be differential or internal (e.g.: TEMPSENSE, DAC, REF0...)
        switch (cal_loaded)
        {
        case CALIBRATE_LOADED_NO:
            ch = ADC_CH_ADC0;
            adcConfMap[ch].calibration = 1.0/4096;
            adcConfMap[ch].ref = &ref_extern_vdd;
            adcConfMap[ch].adc0ref = VREF_REFSEL_VDD_gc;
            adcConfMap[ch].muxpos = ADC_MUXPOS_AIN0_gc;
            adcConfMap[ch].muxneg = ADC_MUXNEG_GND_gc;
            adcConfMap[ch].sampctrl = 0;
            if (i2c_success) cal_loaded = CALIBRATE_LOADED_CH0;
            break;
        case CALIBRATE_LOADED_CH0:
            ch = ADC_CH_ADC1;
            adcConfMap[ch].calibration = 1.0/4096;
            adcConfMap[ch].ref = &ref_extern_vdd;
            adcConfMap[ch].adc0ref = VREF_REFSEL_VDD_gc;
            adcConfMap[ch].muxpos = ADC_MUXPOS_AIN1_gc;
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
            ch = ADC_CH_ADC5;
            adcConfMap[ch].calibration = 1.0/4096;
            adcConfMap[ch].ref = &ref_extern_vdd;
            adcConfMap[ch].adc0ref = VREF_REFSEL_VDD_gc;
            adcConfMap[ch].muxpos = ADC_MUXPOS_AIN5_gc;
            adcConfMap[ch].muxneg = ADC_MUXNEG_GND_gc;
            adcConfMap[ch].sampctrl = 0;
            if (i2c_success) cal_loaded = CALIBRATE_LOADED_CH5;
            break;
        case CALIBRATE_LOADED_CH5:
            ch = ADC_CH_ADC6;
            adcConfMap[ch].calibration = 1.0/4096;
            adcConfMap[ch].ref = &ref_extern_vdd;
            adcConfMap[ch].adc0ref = VREF_REFSEL_VDD_gc;
            adcConfMap[ch].muxpos = ADC_MUXPOS_AIN6_gc;
            adcConfMap[ch].muxneg = ADC_MUXNEG_GND_gc;
            adcConfMap[ch].sampctrl = 0;
            if (i2c_success) cal_loaded = CALIBRATE_LOADED_CH6;
            break;
        case CALIBRATE_LOADED_CH6:
            ch = ADC_CH_ADC7;
            adcConfMap[ch].calibration = 1.0/4096;
            adcConfMap[ch].ref = &ref_extern_vdd;
            adcConfMap[ch].adc0ref = VREF_REFSEL_VDD_gc;
            adcConfMap[ch].muxpos = ADC_MUXPOS_AIN7_gc;
            adcConfMap[ch].muxneg = ADC_MUXNEG_GND_gc;
            adcConfMap[ch].sampctrl = 0;
            if (i2c_success) cal_loaded = CALIBRATE_LOADED_CH7;
            break;
        case CALIBRATE_LOADED_CH7:
            cal_loaded = CALIBRATE_LOADED_DONE;
            break;

        default:
            cal_loaded = CALIBRATE_LOADED_ERR;
            break;
        }
        return false;
    }
    if (cal_mgr_loaded < CAL_MGR_LOADED_DONE) {
        uint16_t i2c_success = 1; // i2c will take several scan loops to finish each cal. It is a place holder for the code.
        ADC_CH_t ch; // channels may be differential or internal (TEMPSENSE, DAC, REF0...)
        switch (cal_mgr_loaded)
        {
        case CAL_MGR_LOADED_NO:
            ch = ADC_CH_MGR_ALT_I;
            adcMgrConfMap[ch].calibration = (1.0/4096)/(0.018*50.0);
            adcMgrConfMap[ch].ref = &ref_extern_vdd;
            if (i2c_success) cal_mgr_loaded = CAL_MGR_LOADED_ALT_I;
            break;
        case CAL_MGR_LOADED_ALT_I:
            ch = ADC_CH_MGR_ALT_V;
            adcMgrConfMap[ch].calibration = (110.0/10.0)/4096;
            adcMgrConfMap[ch].ref = &ref_extern_vdd;
            if (i2c_success) cal_mgr_loaded = CAL_MGR_LOADED_ALT_V;
            break;
        case CAL_MGR_LOADED_ALT_V:
            ch = ADC_CH_MGR_PWR_I;
            adcMgrConfMap[ch].calibration = (1.0/4096)/(0.068*50.0);
            adcMgrConfMap[ch].ref = &ref_extern_vdd;
            if (i2c_success) cal_mgr_loaded = CAL_MGR_LOADED_PWR_I;
            break;
        case CAL_MGR_LOADED_PWR_I:
            ch = ADC_CH_MGR_PWR_V;
            adcMgrConfMap[ch].calibration = (115.8/15.8)/4096;
            adcMgrConfMap[ch].ref = &ref_extern_vdd;
            if (i2c_success) cal_mgr_loaded = CAL_MGR_LOADED_PWR_V;
            break;
        case CAL_MGR_LOADED_PWR_V:
            cal_mgr_loaded = CAL_MGR_LOADED_DONE;
            break;

        default:
            cal_mgr_loaded = CAL_MGR_LOADED_ERR;
            break;
        }
        return false;
    }
    return true; // well that was is a lot of stuff to load 
}
