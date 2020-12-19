/*
ee is a library that has commands to work with the EEPROM. 
Copyright (C) 2019 Ronald Sutherland

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <util/atomic.h>
// the new eeprom.h from Microchip 3.6.2 (also the newer mplab 2.31 source for GCC)
// does not work
// the new Debain package has 3.6.2
// https://salsa.debian.org/debian/avr-libc/-/blob/09632ba9991dc8f7ad6d208b6c946940eaefc29e/libc/avr-libc/include/avr/eeprom.h
// its eewr_block_xmega.c is setup for m4809, but not AVR128Dx
// https://salsa.debian.org/debian/avr-libc/-/blob/09632ba9991dc8f7ad6d208b6c946940eaefc29e/libc/avr-libc/libc/misc/eewr_block_xmega.c
// El Tangas post a link for the 3.6.2 source
// https://www.avrfreaks.net/comment/3052341#comment-3052341
// gchapman post a link for mplab 2.31 source for GCC, mplab has both xc8 and GCC compilers (xc8 is not gpl).
// https://www.avrfreaks.net/comment/3023176#comment-3023176
// which changes eewr_block_xmega.c a little. Basicly turning everything in it off for Dx parts.
// That made me look a little at the .S files where I found eerd_byte.S and eewr_byte.S, but those are set up for m4809.
// So I am doing what curtvm shows for this header.
// https://www.avrfreaks.net/comment/3053261#comment-3053261
#include "../lib/eerw_dx.h"
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>
#include "../lib/parse.h"
#include "ee.h"

static uint32_t ee_mem;

uint8_t ee_read_type(const char * addr, const char * type)
{
    if ( (type == NULL) || (strcmp_P(type, PSTR("UINT8")) ==0) )
    {
        ee_mem = (uint32_t) eeprom_read_byte( (uint8_t*)(atoi(addr)) );
        return 1;
    }
    if ( strcmp_P(type, PSTR("UINT16")) == 0 )
    {
        ee_mem =(uint32_t) eeprom_read_word((uint16_t*)(atoi(addr)));
        return 1;
    }
    if ( strcmp_P(type, PSTR("UINT32")) == 0 )
    {
        ee_mem =(uint32_t) eeprom_read_dword((uint32_t*)(atoi(addr)));
        return 1;
    }
    return 0;
}

/* /0/ee? 0..1023, [UINT8|UINT16|UINT32] */
void EEread_cmd(void)
{
    if (arg_count > 2)
    {
        printf_P(PSTR("{\"err\":\"EeRdArgCount\"}\r\n"));
        initCommandBuffer();
        return;
    }
    
    if ( (command_done == 10) )
    {
        // check that argument[0] is in the range 0..1023
        if ( ( !( isdigit(arg[0][0]) ) ) || (atoi(arg[0]) < 0) || (atoi(arg[0]) >= EEPROM_SIZE) )
        {
            printf_P(PSTR("{\"err\":\"EeRdMaxAddr %d\"}\r\n"), EEPROM_SIZE);
            initCommandBuffer();
            return;
        }

        if ( arg_count == 1)
        {
            if (arg[1] != NULL)
            {
                printf_P(PSTR("{\"err\":\"ParserBroken\"}\r\n"));
                initCommandBuffer();
                return;
            }
        }

       // check that argument[1] is UINT8|UINT16|UINT32
        if ( ( arg_count == 2 ) && ( !( (strcmp_P(arg[1], PSTR("UINT8")) == 0) || \
                                                    (strcmp_P(arg[1], PSTR("UINT16")) == 0) || \
                                                    (strcmp_P(arg[1], PSTR("UINT32")) == 0) ) ) )
        {
            printf_P(PSTR("{\"err\":\"EeRdTypUINT8|16|32\"}\r\n"));
            initCommandBuffer();
            return;
        }
        
        printf_P(PSTR("{\"EE[%s]\":{"),arg[0]);
        ee_mem = 0;
        command_done = 11;
    }
    else if ( (command_done == 11) )
    {  // I don't think there is much blocking during the EEPROM read.
        if (!ee_read_type(arg[0], arg[1]))
        {
            printf_P(PSTR("\"err\":\"EeRdCmdDn11WTF\"}}\r\n"));
            initCommandBuffer();
            return;
        }
        command_done = 12;
    }
    else if ( (command_done == 12) )
    {
        printf_P(PSTR("\"r\":\"%lu\"}}\r\n"),ee_mem);
        initCommandBuffer();
    }
    else
    {
        printf_P(PSTR("{\"err\":\"EeCmdDoneWTF\"}\r\n"));
        initCommandBuffer();
    }
}

/* /0/ee address,value[,type] */
void EEwrite_cmd(void)
{
    if ( (command_done == 10) )
    {
        // check that argument[0] is in the range 0..EEPROM_SIZE
        if ( ( !( isdigit(arg[0][0]) ) ) || (atoi(arg[0]) < 0) || (atoi(arg[0]) >= EEPROM_SIZE) )
        {
            printf_P(PSTR("{\"err\":\"EeAddrSize %d\"}\r\n"), EEPROM_SIZE);
            initCommandBuffer();
            return;
        }
        
        // check that argument[1] is a number (it will not overflow a uint32_t)
        if ( !( isdigit(arg[1][0]) ) )
        {
            printf_P(PSTR("{\"err\":\"EeData!=uint8_t\"}\r\n"));
            initCommandBuffer();
            return;
        }
        ee_mem = strtoul(arg[1], (char **)NULL, 10);

        if ( arg_count == 2)
        {
            if (arg[2] != NULL)
            {
                printf_P(PSTR("{\"err\":\"ParserBroken\"}\r\n"));
                initCommandBuffer();
                return;
            }
        }
        
       // check that argument[2] is UINT8|UINT16|UINT32
        if ( ( arg_count == 3 ) && ( !( (strcmp_P(arg[2], PSTR("UINT8")) == 0) || \
                                                    (strcmp_P(arg[2], PSTR("UINT16")) == 0) || \
                                                    (strcmp_P(arg[2], PSTR("UINT32")) == 0) ) ) )
        {
            printf_P(PSTR("{\"err\":\"EeWrTypUINT8|16|32\"}\r\n"));
            initCommandBuffer();
            return;
        }
        
        printf_P(PSTR("{\"EE[%d]\":{"), atoi(arg[0]));
        command_done = 11;
    }
    else if ( (command_done == 11) )
    {  
        // add a check if we can use eeprom.
        /*if ( eeprom_is_ready() ) 
        {
        */
            if ( (arg[2] == NULL) || (strcmp_P(arg[2], PSTR("UINT8")) == 0) )
            {
                uint8_t value = (uint8_t) (ee_mem & 0xFFU);
                printf_P(PSTR("\"byte\":\"%u\","),value);
                eeprom_write_byte( (uint8_t *) (atoi(arg[0])), value);
            }
/*
            if ( strcmp_P(arg[2], PSTR("UINT16")) == 0 )
            {
                uint16_t value = (uint16_t) (ee_mem & 0xFFFFU);
                printf_P(PSTR("\"word\":\"%u\","),value);
                eeprom_write_word( (uint16_t *) (atoi(arg[0])), value);
            }
            if ( strcmp_P(arg[2], PSTR("UINT32")) == 0 )
            {
                printf_P(PSTR("\"dword\":\"%lu\","),ee_mem);
                eeprom_write_dword( (uint32_t *) (atoi(arg[0])), ee_mem);
            }
*/
            command_done = 12;
        /*
        }
        */
    }
    else if ( (command_done == 12) )
    {
        if (!ee_read_type(arg[0], arg[2]))
        {
            printf_P(PSTR("{\"err\":\"EeWrCmdDn12WTF\"}\r\n"));
            initCommandBuffer();
            return;
        }
        command_done = 13;
    }
    else if ( (command_done == 13) )
    {
        printf_P(PSTR("\"r\":\"%lu\"}}\r\n"),ee_mem);
        initCommandBuffer();
    }
    else
    {
        printf_P(PSTR("{\"err\":\"AdcCmdDoneWTF\"}\r\n"));
        initCommandBuffer();
    }
}

