#ifndef References_H
#define References_H

#include <stdbool.h>

typedef enum VREF_LOADED_enum
{
    VREF_LOADED_NO,  // not loaded and not started
    VREF_LOADED_VDD,  // work in prcess, VDD loaded
    VREF_LOADED_1V0,  // work in prcess, 1V0 loaded
    VREF_LOADED_2V0,  // work in prcess, 2V0 loaded
    VREF_LOADED_4V1,  // work in prcess, 4V1 loaded
    VREF_LOADED_DONE,  // references loaded
    VREF_LOADED_ERR  // Fail to load referances
} VREF_LOADED_t;

typedef enum CALIBRATE_LOADED_enum
{
    CALIBRATE_LOADED_NO,  // not loaded and not started
    CALIBRATE_LOADED_CH0,  // work in prcess, load channel 0 and set reference
    CALIBRATE_LOADED_CH1,  // work in prcess, load channel 1 and set reference
    CALIBRATE_LOADED_CH2,  // work in prcess, load channel 2 and set reference
    CALIBRATE_LOADED_CH3,  // work in prcess, load channel 3 and set reference
    CALIBRATE_LOADED_CH4,  // work in prcess, load channel 4 and set reference
    CALIBRATE_LOADED_CH5,  // work in prcess, load channel 5 and set reference
    CALIBRATE_LOADED_CH6,  // work in prcess, load channel 6 and set reference
    CALIBRATE_LOADED_CH7,  // work in prcess, load channel 7 and set reference
    CALIBRATE_LOADED_DONE,  // calibrations loaded and references set
    CALIBRATE_LOADED_ERR  // Fail to load calibrations
} CALIBRATE_LOADED_t;

typedef enum CAL_MGR_LOADED_enum
{
    CAL_MGR_LOADED_NO,  // not loaded and not started
    CAL_MGR_LOADED_ALT_I,  // work in prcess, load ALT_I cal and reference
    CAL_MGR_LOADED_ALT_V,  // work in prcess, load ALT_V cal and reference
    CAL_MGR_LOADED_PWR_I,  // work in prcess, load load PWR_I cal and reference
    CAL_MGR_LOADED_PWR_V,  // work in prcess, load load PWR_V cal and reference
    CAL_MGR_LOADED_DONE,  // calibrations loaded and references set
    CAL_MGR_LOADED_ERR  // Fail to load calibrations
} CAL_MGR_LOADED_t;

extern VREF_LOADED_t ref_loaded;
extern float ref_extern_vdd;
extern float ref_intern_1v0;
extern float ref_intern_2v0;
extern float ref_intern_4v1;

extern CALIBRATE_LOADED_t cal_loaded;
extern CAL_MGR_LOADED_t cal_mgr_loaded;

struct AdcConf_Map { 
    float calibration; // calibrated value need for an ADC divion of the channel.
    float *ref; // pointer to the referance value used for channel
    VREF_REFSEL_t adc0ref; // Setting for ADC0 Reference register
    ADC_MUXPOS_t muxpos; // Setting for ADC0 Positive mux input register
    ADC_MUXNEG_t muxneg; // Setting for ADC0 Negative mux input register
    uint8_t sampctrl; // Extend the ADC sampling time beyond the default two clocks
};

extern struct AdcConf_Map adcConfMap[]; // size is ADC_CHANNELS

struct AdcMgrConf_Map {
    int value; // ADC value from manager
    float calibration; // calibrated value need for an ADC divion of the channel.
    float *ref; // pointer to the referance value used for channel
};

extern struct AdcMgrConf_Map adcMgrConfMap[]; // size is 4 (alt_i, alt_v, pwr_i, pwr_v)

extern bool LoadAdcConfig();

#endif // Analog_H 
