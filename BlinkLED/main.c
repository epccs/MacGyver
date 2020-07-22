/* Blink LED
Copyright (C) 2019 Ronald Sutherland

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
#include <util/delay.h>
#include <util/atomic.h>
#include <avr/io.h>
#include "../lib/io_enum_bsd.h"
//#include "../lib/twi0_mc.h"

/* cycle the twi state machine on both the master and slave(s)
void i2c_ping(void)
{ 
    // ping I2C for an RPU bus manager 
    uint8_t i2c_address = 41; //I2C_ADDR_OF_BUS_MGR
    uint8_t data = 0;
    uint8_t length = 0;
    // uint8_t wait = 1;
    uint8_t sendStop = 1;
    for (uint8_t i =0;1; i++) // try a few times.
    {
        uint8_t twi_errorCode = TWI_MasterWrite(i2c_address, &data, length, sendStop); 
        if (twi_errorCode == 0) break; // ping was error free
        if (i>5) return; // give up after 5 trys
    }
    return; 
}
*/

int main(void)
{
    ioCntl(MCU_IO_AIN0, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    ioDir(MCU_IO_AIN0, DIRECTION_OUTPUT);
    ioWrite(MCU_IO_AIN0,LOGIC_LEVEL_HIGH);

    // init i2c master with normal 100k clock
    //TWI_MasterInit(100000UL);

    // Enable global interrupts to start I2C
    sei();

    while (1)
    {
        _delay_ms(500); // need to do Timer0 OVF tick clock so I don't have to block
        ioToggle(MCU_IO_AIN0);
        //i2c_ping(); // That poor I2C address is going to lose it.
    }
}

