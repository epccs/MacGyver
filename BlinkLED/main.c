/* Blink LED
Copyright (C) 2019 Ronald Sutherland

This program is free software; you can redistribute it and/or
modify it under the terms of the GNU General Public License
as published by the Free Software Foundation; either version 2
of the License, or (at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

For a copy of the GNU General Public License use
http://www.gnu.org/licenses/gpl-2.0.html
 */ 

#include <util/delay.h>
#include <avr/io.h>
#include <stdbool.h>
#include "../lib/pin_num.h"

int main(void)
{
    pinMode(0,OUTPUT);
    digitalWrite(0,HIGH);

    while (1)
    {
	    _delay_ms(500);
	    digitalToggle(0);
    }
}

