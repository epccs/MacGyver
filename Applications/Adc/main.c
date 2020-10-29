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
#include "../lib/twi0_bsd.h"
#include "../lib/rpu_mgr.h"
#include "../lib/io_enum_bsd.h"
#include "../Uart/id.h"
#include "analog.h"

#define ADC_DELAY_MILSEC 200UL
static unsigned long adc_started_at;

#define BLINK_DELAY 1000UL
static unsigned long blink_started_at;
static unsigned long blink_delay;
static char rpu_addr;

void ProcessCmd()
{ 
    if ( (strcmp_P( command, PSTR("/id?")) == 0) && ( (arg_count == 0) || (arg_count == 1)) )
    {
        Id("Adc");
    }
    if ( (strcmp_P( command, PSTR("/analog?")) == 0) && ( (arg_count >= 1 ) && (arg_count <= 5) ) )
    {
        Analogf(cnvrt_milli(2000UL)); // update every 2 sec until terminated
    }
    if ( (strcmp_P( command, PSTR("/adc?")) == 0) && ( (arg_count >= 1 ) && (arg_count <= 5) ) )
    {
        Analogd(cnvrt_milli(2000UL)); // update every 2 sec until terminated
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
    
    // Initialize Timers, ADC, and clear bootloader, Arduino does these with init() in wiring.c
    initTimers(); //Timer0 Fast PWM mode, Timer1 & Timer2 Phase Correct PWM mode.
    init_ADC_single_conversion();

    // put ADC in Auto Trigger mode and fetch an array of channels
    //enable_ADC_auto_conversion(BURST_MODE);
    adc_started_at = milliseconds();

    /* Initialize UART to 38.4kbps, it returns a pointer to FILE so redirect of stdin and stdout works*/
    stderr = stdout = stdin = uart0_init(38400UL, UART0_RX_REPLACE_CR_WITH_NL);
    
    /* Initialize I2C */
    twi0_init(100000UL, TWI0_PINS_PULLUP);

    /* Clear and setup the command buffer, (probably not needed at this point) */
    initCommandBuffer();

    // Enable global interrupts to start TIMER0 and UART ISR's
    sei(); 
    
    blink_started_at = milliseconds();
    
    rpu_addr = i2c_get_Rpu_address();
    blink_delay = BLINK_DELAY;
    
    // blink fast if a default address from RPU manager not found
    if (rpu_addr == 0)
    {
        rpu_addr = '0';
        blink_delay = BLINK_DELAY/4;
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

void adc_burst(void)
{
    unsigned long kRuntime= elapsed(&adc_started_at);
    if ((kRuntime) > ((unsigned long)ADC_DELAY_MILSEC))
    {
        //enable_ADC_auto_conversion(BURST_MODE);
        adc_started_at += ADC_DELAY_MILSEC; 
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
            AssembleCommand(getchar());

            // address is an ascii value, warning: a null address would terminate the command string. 
            StartEchoWhenAddressed(rpu_addr);
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
        adc_burst();
          
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
                    findCommand();
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
