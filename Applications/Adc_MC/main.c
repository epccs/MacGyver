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
ADC_MUXPOS_t adc_ch;

void PORT_init(void);
void ADC0_init(void);
uint16_t ADC0_read(void);
void ADC0_start(ADC_MUXPOS_t mux);

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
    VREF.ADC0REF = VREF_REFSEL_VDD_gc;     /* reference */
    ADC0.CTRLC = ADC_PRESC_DIV4_gc;        /* CLK_PER divided by 4 (e.g. F_CPU/4 = 1 MHz)*/    
    ADC0.CTRLA = ADC_ENABLE_bm             /* ADC Enable: enabled */
               | ADC_RESSEL_12BIT_gc;      /* 12-bit mode */
}

uint16_t ADC0_read(void)
{
    /* Clear the interrupt flag by reading the result */
    return ADC0.RES;
}

void ADC0_start(ADC_MUXPOS_t mux)
{
    ADC0.MUXPOS = mux;               /* Select ADC channel */
    ADC0.COMMAND = ADC_STCONV_bm;    /* Start ADC conversion */
}

int main(void)
{
    PORT_init();
    ADC0_init();
    adc_ch = ADC_MUXPOS_AIN0_gc;
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
                if (adc_ch >= ADC_MUXPOS_AIN7_gc)
                {
                    printf_P(PSTR("\r\n"));
                    adc_ch = ADC_MUXPOS_AIN0_gc;
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


