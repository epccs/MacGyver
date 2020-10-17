#ifndef AdcISR_h
#define AdcISR_h

#include <stdint.h>

// Analog values range from 0 to 1023, they have 1024 slots where each 
// reperesents 1/1024 of the reference. The last slot has issues see datasheet.
// the ADC channel can be used with analogRead(ADC0)*(<referance>/1024.0)

// enumeraiton names for ADC_CH_<node> from schematic
typedef enum ADC_CH_enum {
    ADC_CH_ADC0, // PA0 has analog channel 0
    ADC_CH_ADC1, // PA1 has analog channel 1
    ADC_CH_ADC2, // PA2 has analog channel 2
    ADC_CH_ADC3, // PA3 has analog channel 3
    ADC_CH_ADC4, // PA4 has analog channel 4
    ADC_CH_ADC5, // PA5 has analog channel 5
    ADC_CH_ADC6, // PA6 has analog channel 6
    ADC_CH_ADC7, // PA7 has analog channel 7
    ADC_CHANNELS
} ADC_CH_t;

extern volatile int adc[];
extern volatile uint8_t adc_channel;
extern volatile uint8_t ADC_auto_conversion;
extern volatile uint8_t analog_reference;

#define ISR_ADCBURST_DONE 0x7F
#define ISR_ADCBURST_START 0x00
extern volatile uint8_t adc_isr_status;

/* ioavr128da... has a reference select enum that may be passed to init_ADC_single_conversion
typedef enum VREF_REFSEL_enum
{
    VREF_REFSEL_1V024_gc = (0x00<<0),  // Internal 1.024V reference
    VREF_REFSEL_2V048_gc = (0x01<<0),  // Internal 2.048V reference
    VREF_REFSEL_4V096_gc = (0x02<<0),  // Internal 4.096V reference
    VREF_REFSEL_2V500_gc = (0x03<<0),  // Internal 2.500V reference
    VREF_REFSEL_VDD_gc = (0x05<<0),  // VDD as reference
    VREF_REFSEL_VREFA_gc = (0x06<<0),  // External referance on VREFA pin
} VREF_REFSEL_t;
*/

#ifndef EXTERNAL_AVCC
#   error your mcu is not supported
#endif
extern void init_ADC_single_conversion(VREF_REFSEL_t reference);
extern int adcAtomic(ADC_CH_t channel);
extern int adcSingle(uint8_t channel);

#define FREE_RUNNING 1
#define BURST_MODE 0
extern void enable_ADC_auto_conversion(uint8_t free_run);

#endif // AdcISR_h
