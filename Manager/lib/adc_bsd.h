#ifndef AdcISR_h
#define AdcISR_h

#include <stdint.h>

// Analog values range from 0 to 4095, they have 4096 slots where each 
// reperesents 1/4096 of the reference. The last slot has issues see datasheet.
// the ADC channel can be used with analogRead(ADC0)*(<referance>/4096.0)

// enumeraiton names for ADC_CH_<node> from schematic
typedef enum ADC_CH_enum {
    ADC_CH_ADC1 = 1, // this channels measures AIN1 to GND, AIN1 is on pin PD1
    ADC_CH_ADC2, // this channels measures AIN2 to GND, AIN2 is on pin PD2
    ADC_CH_ADC3, // this channels measures AIN3 to GND, AIN3 is on pin PD3
    ADC_CH_ADC4, // this channels measures AIN4 to GND, AIN4 is on pin PD4
    ADC_CHANNELS
} ADC_CH_t;

extern volatile int adc[];
extern volatile ADC_CH_t adc_channel;
extern volatile VREF_REFSEL_t analog_reference;

#define ISR_ADCBURST_DONE 0x7F
#define ISR_ADCBURST_START 0x00
extern volatile uint8_t adc_isr_status;
extern uint8_t adc_auto_conversion; // don't do single conversions when auto_conversion is running

extern void init_ADC_single_conversion(void);
extern int adcAtomic(ADC_CH_t channel);
extern int adcSingle(ADC_CH_t channel);

#define FREE_RUNNING 1
#define BURST_MODE 0
extern void enable_ADC_auto_conversion(uint8_t free_run);

#endif // AdcISR_h
