/*
id is a library that returns a name, description and tool.
Copyright (C) 2019 Ronald Sutherland

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
#include <avr/pgmspace.h>
#include <stdio.h>
#include <stdlib.h> 
#include "../lib/parse.h"
#include "../lib/rpu_mgr.h"
#include "id.h"

void Id(char name[])
{ 
    // /id? 
    if ( (command_done == 10) && (arg_count == 0) )
    {
        printf_P(PSTR("{\"id\":{"));
        command_done = 11;
    }
    // /id? name 
    else if ( (command_done == 10) && (arg_count == 1) && (strcmp_P( arg[0], PSTR("name")) == 0) ) 
    {
        printf_P(PSTR("{\"id\":{"));
        command_done = 11;
    }
    // /id? desc
    else if ( (command_done == 10) && (arg_count == 1) && (strcmp_P( arg[0], PSTR("desc")) == 0) )
    {
        printf_P(PSTR("{\"id\":{" ));
        command_done = 12;
    }
    // /id? avr-gcc
    else if ( (command_done == 10) && (arg_count == 1) && (strcmp_P( arg[0], PSTR("avr-gcc")) == 0) )
    {
        printf_P(PSTR("{\"id\":{"));
        command_done = 14;
    }
    else if ( command_done == 11 )
    {
        printf_P(PSTR("\"name\":\"%s\"" ),name);
        if (arg_count == 1) 
        { 
            command_done = 15;  
        }
        else 
        { 
            printf_P(PSTR("," ));
            command_done = 12; 
        }
    }
    else if ( command_done == 12 )
    {
        printf_P(PSTR("\"desc\":\"MacGyver (19260^1) " ));
        command_done = 13;
    }
    else if ( command_done == 13 )
    {
        printf_P(PSTR("Board /w AVR128DA28\""));
        if (arg_count == 1) 
        { 
            command_done = 15; 
        }
        else 
        { 
            printf_P(PSTR("," ));
            command_done = 14; 
        }
    }
    else if ( command_done == 14 )
    {
        printf_P(PSTR("\"avr-gcc\":\"%s\""),__VERSION__);
        command_done = 15; 
    }
    else if ( command_done == 15 )
    {
        printf_P(PSTR("}}\r\n"));
        initCommandBuffer();
    }
    else
    {
        printf_P(PSTR("{\"err\":\"idBadArg_%s\"}\r\n"),arg[0]);
        initCommandBuffer();
    }
}

