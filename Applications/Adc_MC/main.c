/*
    \file   main.c

    \brief  ADC Single Conversion

    (c) 2019 Microchip Technology Inc. and its subsidiaries.

    Subject to your compliance with these terms, you may use Microchip software and any
    derivatives exclusively with Microchip products. It is your responsibility to comply with third-party
    license terms applicable to your use of third-party software (including open source software) that
    may accompany Microchip software.

    THIS SOFTWARE IS SUPPLIED BY MICROCHIP "AS IS". NO WARRANTIES, WHETHER
    EXPRESS, IMPLIED OR STATUTORY, APPLY TO THIS SOFTWARE, INCLUDING ANY
    IMPLIED WARRANTIES OF NON-INFRINGEMENT, MERCHANTABILITY, AND FITNESS
    FOR A PARTICULAR PURPOSE.

    IN NO EVENT WILL MICROCHIP BE LIABLE FOR ANY INDIRECT, SPECIAL, PUNITIVE,
    INCIDENTAL OR CONSEQUENTIAL LOSS, DAMAGE, COST OR EXPENSE OF ANY KIND
    WHATSOEVER RELATED TO THE SOFTWARE, HOWEVER CAUSED, EVEN IF MICROCHIP
    HAS BEEN ADVISED OF THE POSSIBILITY OR THE DAMAGES ARE FORESEEABLE. TO
    THE FULLEST EXTENT ALLOWED BY LAW, MICROCHIP'S TOTAL LIABILITY ON ALL
    CLAIMS IN ANY WAY RELATED TO THIS SOFTWARE WILL NOT EXCEED THE AMOUNT
    OF FEES, IF ANY, THAT YOU HAVE PAID DIRECTLY TO MICROCHIP FOR THIS
    SOFTWARE.
*/

#include <avr/io.h>
#include <stdbool.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "../lib/uart0_bsd.h"
#include "../lib/io_enum_bsd.h"

uint16_t adcVal;
uint8_t adc_print_per_line;

// enumeraiton names for ADC_CH_<node> from schematic
typedef enum ADC_CH_enum {
    ADC_CH_ADC0, // this channels measures AIN0 to GND, AIN0 is on pin PD0
    ADC_CH_ADC1, // this channels measures AIN1 to GND, AIN1 is on pin PD1
    ADC_CH_ADC2, // this channels measures AIN2 to GND, AIN2 is on pin PD2
    ADC_CH_ADC3, // this channels measures AIN3 to GND, AIN3 is on pin PD3
    ADC_CH_ADC4, // this channels measures AIN4 to GND, AIN4 is on pin PD4
    ADC_CH_ADC5, // this channels measures AIN5 to GND, AIN5 is on pin PD5
    ADC_CH_ADC6, // this channels measures AIN6 to GND, AIN6 is on pin PD6
    ADC_CH_ADC7, // this channels measures AIN7 to GND, AIN7 is on pin PD7
    ADC_CHANNELS
} ADC_CH_t;

ADC_CH_t adc_ch;

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

struct AdcConf_Map { 
    float calibration; // calibrated value need for an ADC divion of the channel.
    float *ref; // pointer to the referance value used for channel
    VREF_REFSEL_t adc0ref; // Setting for ADC0 Reference register
    ADC_MUXPOS_t muxpos; // Setting for ADC0 Positive mux input register
    ADC_MUXNEG_t muxneg; // Setting for ADC0 Negative mux input register
    uint8_t sampctrl; // Extend the ADC sampling time beyond the default two clocks
};

CALIBRATE_LOADED_t cal_loaded;
struct AdcConf_Map adcConfMap[ADC_CHANNELS]; // size is ADC_CHANNELS
float ref_extern_vdd;

// Load the hardware setting for ADC channel operation
// TODO: load calibrations from manager over I2C
CALIBRATE_LOADED_t LoadAdcConfig()
{
    uint16_t i2c_success = 1; // i2c will take several scan loops to finish each cal. It is a place holder for the code.
    ADC_CH_t ch; // channels may be differential or internal things (TEMPSENSE, DAC, REF0...)
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

    return cal_loaded;
}

void PORT_init(void)
{
    // Disable pull-up and Disable interrupt and digital input buffer on PD[0..7]
    ioDir(MCU_IO_AIN0, DIRECTION_INPUT);
    ioCntl(MCU_IO_AIN0, PORT_ISC_INPUT_DISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_AIN1, DIRECTION_INPUT);
    ioCntl(MCU_IO_AIN1, PORT_ISC_INPUT_DISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_AIN2, DIRECTION_INPUT);
    ioCntl(MCU_IO_AIN2, PORT_ISC_INPUT_DISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_AIN3, DIRECTION_INPUT);
    ioCntl(MCU_IO_AIN3, PORT_ISC_INPUT_DISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_AIN4, DIRECTION_INPUT);
    ioCntl(MCU_IO_AIN4, PORT_ISC_INPUT_DISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_AIN5, DIRECTION_INPUT);
    ioCntl(MCU_IO_AIN5, PORT_ISC_INPUT_DISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_AIN6, DIRECTION_INPUT);
    ioCntl(MCU_IO_AIN6, PORT_ISC_INPUT_DISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_AIN7, DIRECTION_INPUT);
    ioCntl(MCU_IO_AIN7, PORT_ISC_INPUT_DISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
}

void ADC0_init(void)
{
    ADC0.CTRLC = ADC_PRESC_DIV4_gc;        // CLK_PER divided by 4 (e.g. F_CPU/4 = 1 MHz)
    ADC0.CTRLA = ADC_ENABLE_bm             // ADC Enable: enabled
               | ADC_RESSEL_12BIT_gc;      // 12-bit mode
    ADC0.CTRLD = ADC_INITDLY_DLY16_gc;     // let referance settle

    // load config and calibration or set error
    cal_loaded = CALIBRATE_LOADED_NO;
    while(cal_loaded < CALIBRATE_LOADED_DONE)
    {
        LoadAdcConfig();
    }
}

uint16_t ADC0_read(void)
{
    /* Clear the interrupt flag by reading the result */
    return ADC0.RES;
}

void ADC0_start(ADC_CH_t ch)
{
    VREF.ADC0REF = adcConfMap[ch].adc0ref;  /* Select referance */
    ADC0.MUXPOS = adcConfMap[ch].muxpos;    /* Select ADC channel */
    ADC0.COMMAND = ADC_STCONV_bm;           /* Start ADC conversion */
}

int main(void)
{
    ref_extern_vdd = 5.0;
    PORT_init();
    ADC0_init();
    adc_ch = ADC_CH_ADC0;
    ADC0_start(adc_ch);

    /* Initialize UART, it returns a pointer to FILE so redirect of stdin and stdout works*/
    stderr = stdout = stdin = uart0_init(9600UL, UART0_RX_REPLACE_CR_WITH_NL);

    // Enable global interrupts to start UART ISR
    sei();

    while (1)
    {
        if (ADC0.INTFLAGS & ADC_RESRDY_bm) // Check if the conversion is done
        {
            if (uart0_availableForWrite()) 
            {
                adcVal = ADC0_read();
                printf_P(PSTR("  %d\t"), adcVal);
                if (adc_ch >= ADC_CH_ADC7)
                {
                    printf_P(PSTR("\r\n"));
                    adc_ch = ADC_CH_ADC0;
                }
                else
                {
                    adc_ch++;
                }
                ADC0_start(adc_ch); // Start the next ADC conversion
            }
        }
    }
}


