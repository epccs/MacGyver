/*
Uart is a demonstration of an Interrupt-Driven UART with stdio redirect. 
Copyright (C) 2016 Ronald Sutherland

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
#include <avr/interrupt.h>
#include "../lib/timers_bsd.h"
#include "../lib/uart0_bsd.h"
#include "../lib/parse.h"
#include "../lib/twi.h"
#include "../lib/rpu_mgr.h"
#include "../lib/io_enum_bsd.h"
#include "id.h"

#define BLINK_DELAY 1000UL
static unsigned long blink_started_at;
static unsigned long blink_delay;
static char rpu_addr;

FILE *uart0;

void ProcessCmd()
{ 
    if ( (strcmp_P( command, PSTR("/id?")) == 0) && ( (arg_count == 0) || (arg_count == 1)) )
    {
        Id(uart0, PSTR("Uart"));
    }
}

void setup(void) 
{
    // STATUS_LED
    ioDir(MCU_IO_TX2, DIRECTION_OUTPUT); 
    ioWrite(MCU_IO_TX2, LOGIC_LEVEL_HIGH);
    
    // Initialize Timers TCA0 is split into two 8 bit timers, the high underflow (HUNF) event it used for  time tracking
    initTimers(); //PWM: TCA route A to PC0, PC1, PC2, PC3, PC4, PC5.

    /* Initialize UART0 to 38.4kbps for streaming, it returns a pointer to a FILE structure */
    uart0 = uart0_init(38400UL, UART0_RX_REPLACE_CR_WITH_NL);

    /* Initialize I2C to manager*/
    twim_defaultPins();           // DA master (and slave) pins are PA2, PA3 and go to the DB (PF2, PF3)
    twim_baud( F_CPU, 100000ul ); // setup the master

    /* Clear and setup the command buffer, (probably not needed at this point) */
    initCommandBuffer();

    // Enable global interrupts to start TIMER0 and UART
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

int main(void) {

    setup(); 

    while(1) 
    {
        // use STATUS_LED to show if I2C has a bus manager
        blink();

        // check if character is available to assemble a command, e.g. non-blocking
        if ( (!command_done) && uart0_available() ) // command_done is an extern from parse.h
        {
            // get a character and use it to assemble a command
            AssembleCommand(uart0);

            // address is a char e.g. the ascii value for '0' warning: a null will terminate the command string. 
            StartEchoWhenAddressed(uart0, rpu_addr);
        }

        // check if the character is available, and if so stop transmit and the command in process.
        // a multi-drop bus can have another device start transmitting after the second received byte so
        // there is little time to detect a possible collision
        if ( command_done && uart0_available() )
        {
            // dump the transmit buffer to limit a collision 
            uart0_empty(); 
            initCommandBuffer();
        }
        
        // finish echo of the command line befor starting a reply (or the next part of reply)
        if ( command_done && uart0_availableForWrite() )
        {
            if ( !echo_on  )
            { // this happons when the address did not match
                initCommandBuffer();
            }
            else
            {
                // command is a pointer to string and arg[] is an array of pointers to strings
                // use findCommand to make them point to the correct places in the command line
                // this can only be done once, since spaces and delimeters are replaced with null termination
                if (command_done == 1)
                {
                    findCommand(uart0);
                    command_done = 10;
                }
                
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
