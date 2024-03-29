/*
Adc is a command line controled demonstration of Interrupt Driven Analog Conversion
Copyright (C) 2016 Ronald Sutherland

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

#include <stdbool.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "../lib/timers_bsd.h"
#include "../lib/uart0_bsd.h"
#include "../lib/parse.h"
#include "../lib/adc_bsd.h"
#include "../lib/twi.h"
#include "../lib/rpu_mgr.h"
#include "../lib/io_enum_bsd.h"
#include "../Uart/id.h"
#include "analog.h"

#define ADC_DELAY_MILSEC 200UL
static unsigned long adc_started_at;
static unsigned long adc_delay;

#define BLINK_DELAY 1000UL
static unsigned long blink_started_at;
static unsigned long blink_delay;
static char rpu_addr;

FILE *uart0;

void ProcessCmd()
{ 
    if ( (strcmp_P( command, PSTR("/id?")) == 0) && ( (arg_count == 0) || (arg_count == 1)) )
    {
        Id(uart0, "Adc");
    }
    if ( (strcmp_P( command, PSTR("/analog?")) == 0) && ( (arg_count >= 1 ) && (arg_count <= 5) ) )
    {
        Analogf(uart0, cnvrt_milli(2000UL)); // update every 2 sec until terminated
    }
    if ( (strcmp_P( command, PSTR("/adc?")) == 0) && ( (arg_count >= 1 ) && (arg_count <= 5) ) )
    {
        Analogd(uart0, cnvrt_milli(2000UL)); // update every 2 sec until terminated
    }
}

void setup(void) 
{
    // To reduce power consumption, the digital input buffer has to be disabled on the pins used as inputs for ADC. 
    // This is configured by the I/O Pin Controller (PORT).
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

    // STATUS_LED
    ioDir(MCU_IO_TX2, DIRECTION_OUTPUT); 
    ioWrite(MCU_IO_TX2, LOGIC_LEVEL_HIGH);

    // Initialize Timers TCA0 is split into two 8 bit timers, the high underflow (HUNF) event it used for  time tracking
    initTimers(); //PWM: TCA route A to PC0, PC1, PC2, PC3, PC4, PC5.

    // ADC, start taking readings on each ADC channel.
    init_ADC_single_conversion();
    adc_started_at = tickAtomic();
    adc_delay = cnvrt_milli(ADC_DELAY_MILSEC);

    /* Initialize UART to 38.4kbps, it returns a pointer to FILE so redirect of stdin and stdout works*/
    uart0 = uart0_init(38400UL, UART0_RX_REPLACE_CR_WITH_NL);

    /* Initialize I2C */
    twim_defaultPins();           // DA master (and slave) pins are PA2, PA3 and go to the DB (PF2, PF3)
    twim_baud( F_CPU, 100000ul ); // setup the master

    /* Clear and setup the command buffer, (probably not needed at this point) */
    initCommandBuffer();

    // Enable global interrupts to start TIMER0 and UART ISR's
    sei(); 

    // tick count is not milliseconds use cnvrt_milli() to convert time into ticks, thus tickAtomic()/cnvrt_milli(1000) gives seconds
    blink_started_at = tickAtomic();
    blink_delay = cnvrt_milli(BLINK_DELAY);

    rpu_addr = i2c_get_Rpu_address();

    // blink fast if a default address from RPU manager not found
    if (rpu_addr == 0)
    {
        rpu_addr = '0';
        blink_delay = blink_delay/4;
    }
}

void blink(void)
{
    unsigned long kRuntime = elapsed(&blink_started_at);
    if ( kRuntime > blink_delay)
    {
        ioToggle(MCU_IO_TX2);

        // next toggle 
        blink_started_at += blink_delay; 
    }
}

int main(void) 
{
    setup();

    while(1) 
    { 
        // use LED to show if I2C has a bus manager
        blink();

        // check if character is available to assemble a command, e.g. non-blocking
        if ( (!command_done) && uart0_available() ) // command_done is an extern from parse.h
        {
            // get a character from stdin and use it to assemble a command
            AssembleCommand(uart0);

            // address is an ascii value, warning: a null address would terminate the command string. 
            StartEchoWhenAddressed(uart0, rpu_addr);
        }

        // check if a character is available, and if so flush transmit buffer and nuke the command in process.
        // A multi-drop bus can have another device start transmitting after getting an address byte so
        // the first byte is used as a warning, it is the onlly chance to detect a possible collision.
        if ( command_done && uart0_available() )
        {
            // dump the transmit buffer to limit a collision 
            uart0_empty(); 
            initCommandBuffer();
        }

        // delay between ADC burst
        adc_burst(&adc_started_at, &adc_delay);

        // finish echo of the command line befor starting a reply (or the next part of a reply)
        if ( command_done && uart0_availableForWrite() )
        {
            if ( !echo_on  )
            { // this happons when the address did not match
                initCommandBuffer();
            }
            else
            {
                if (command_done == 1)
                {
                    findCommand(uart0);
                    command_done = 10;
                }
                
                // do not overfill the serial buffer since that blocks looping, e.g. process a command in 32 byte chunks
                if ( (command_done >= 10) && (command_done < 250) )
                {
                     ProcessCmd();
                }
                else 
                {
                    initCommandBuffer();
                }
            }
         }
    }
    return 0;
}
