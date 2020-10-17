/*
Parse serial commands into tokens e.g. "/0/pwm 252" into "/0/pwm\0" and "252\0" 
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

avr-libc may have some String tokenization in C 
https://onebyezero.blogspot.com/2018/12/string-tokenization-in-c.html
*/
#include <ctype.h>
#include <stdbool.h>
#include <stdlib.h>
#include <avr/pgmspace.h>
#include "parse.h"
#include "uart0_bsd.h"

// used to assemble command line
char command_buf[COMMAND_BUFFER_SIZE];
uint8_t command_head;
uint8_t command_done;

// used to convert command line into its parts
char *command;
char *arg[MAX_ARGUMENT_COUNT];
uint8_t arg_count;

// command loopback happons from the addressed device
uint8_t echo_on;

// Hold the command in the buffer and spin loop until the chunks of JSON 
// are done outputting. Each chunk should be less than 32 bytes since that 
// is the AVR UART buffer size. The main spin loop continues running until 
// the uart is available for write (e.g. buffer is empty) and then the buffer is 
// loaded with the next JSON chunk (a over full buffer will block execution)
void initCommandBuffer(void) 
{
    command_buf[1] = '\0';  // best to set the address as a null value
    for (uint8_t i=0; i < MAX_ARGUMENT_COUNT; i++)
    {
        arg[i] = NULL;
    }
    command = NULL;
    command_done = 0;
    command_head =0;
    arg_count = 0;
    echo_on = 0;
}


void StartEchoWhenAddressed(char address)
{
    if ( (!echo_on) && (command_buf[0] == '/') && (command_buf[1] == address) )
    {
        echo_on = 1;
        printf_P(PSTR("%c%c"),command_buf[0], command_buf[1]);
    }
}

// assemble command line from incoming char's 
void AssembleCommand(int input) 
{
    // a return or new-line finishes the line (or starts a new command line)
    if ( (input == '\r') || (input == '\n') ) // pressing enter in picocom sends a \r
    {
        //echo both carrage return and newline.
        if (echo_on) printf("\r\n");
        
        // finish command line as a null terminated string
        command_buf[command_head] = '\0';
        
        // do not go past the buffer
        if (command_head < (COMMAND_BUFFER_SIZE - 1) )
        {
            ++command_head;
        }
        else // command is to big 
        {
            if (echo_on) printf_P(PSTR("Ignore_Input\r\n"));
            initCommandBuffer();
        }
        command_done = 1;                     
    }
    else
    { 
        if ( (input == '\b') || (input == 0x7F)) // backspace or delete key, picocom maps BS to DEL and DEL to BS by default
        {
            if (command_head>2)
            {
                command_head--; // move pointer back one
                command_buf[command_head] = '\0'; // invalidate
                if (echo_on)
                {
                    putchar ('\b'); // backspace
                    putchar (' '); // space to clear what was
                    putchar ('\b'); // backspace again to position
                }
            }
        }
        else
        {
            //echo the input  
            if (echo_on) putchar(input);

            // assemble the command
            command_buf[command_head] = input;

            // do not go past the buffer
            if (command_head < (COMMAND_BUFFER_SIZE - 1) )
            {
                ++command_head;
            }
            else // command is to big
            {
                command_buf[1] = '\0'; 
                if (echo_on) printf_P(PSTR("Ignore_Input\r\n"));
                echo_on = 0;
            }
        }
    }
}

// find argument(s) starting from a given offset
uint8_t findArgument(uint8_t at_command_buf_offset) 
{
    if (at_command_buf_offset < COMMAND_BUFFER_SIZE) 
    {
        uint8_t lastAlphaNum = at_command_buf_offset;
        
        //get past any white space, but not end of line (EOL was replaced with a null)
        while (isspace(command_buf[lastAlphaNum]) && !(command_buf[lastAlphaNum] == '\0')) 
        { 
            lastAlphaNum++;
        }

        // after command+space but the char is null
        if( (command_buf[lastAlphaNum] == '\0') ) 
        {
            if (echo_on) printf_P(PSTR("{\"err\": \"NullArgAftrCmd+Sp\"}\r\n"));
            initCommandBuffer();
            return 0;
        }
        
        //for each valid argument add it to the arg array of strings
        for (arg_count = 0; command_buf[lastAlphaNum] != '\0' ; arg_count++) 
        {
            // to many arguments
            if( !(arg_count < MAX_ARGUMENT_COUNT) ) 
            {
                if (echo_on) printf_P(PSTR("{\"err\": \"ArgCnt%dAt%d\"}\r\n"), arg_count, lastAlphaNum);
                initCommandBuffer();
                return 0;
            }   
            
            arg[arg_count] = command_buf + lastAlphaNum;
            
            //  skip through the argument
            while( (isalnum(command_buf[lastAlphaNum]) || (command_buf[lastAlphaNum] == '-')) && (lastAlphaNum < (COMMAND_BUFFER_SIZE-1)) ) 
            { 
                lastAlphaNum++;
            }
            if ( (command_buf[lastAlphaNum] == ARGUMNT_DELIMITER) )
            {
                if ( lastAlphaNum < (COMMAND_BUFFER_SIZE-2) ) 
                {
                    // check if char after delimiter is valid for an arg 
                    if( !(isalnum(command_buf[lastAlphaNum+1]) || (command_buf[lastAlphaNum+1] == '-')) ) 
                    {
                        if (echo_on) printf_P(PSTR("{\"err\": \"ArgAftr'%c@%d!Valid\"}\r\n"),command_buf[lastAlphaNum],lastAlphaNum);
                        initCommandBuffer();
                        return 0;
                    }  
                    
                    // null terminate the argument, e.g. replace the delimiter
                    command_buf[lastAlphaNum] = '\0';
                    lastAlphaNum++;
                }
                else
                {
                    // a delimiter was found but there is not enough room for an argument and null termination
                    if (echo_on) printf_P(PSTR("{\"err\": \"DropArgCmdLn2Lng\"}\r\n"));
                    initCommandBuffer();
                    return 0;
                }
            }
            
            // only EOL or delimiter is valid way to terminate an argument (e.g. a space befor end of line is not valid)
            else if (command_buf[lastAlphaNum] != '\0')
            {
                // do not index past command buffer
                if (echo_on) printf_P(PSTR("{\"err\": \"!DelimAftrArg'%c@%d\"}\r\n"), command_buf[lastAlphaNum],lastAlphaNum);
                initCommandBuffer();
                return 0;
            }
        }
        return arg_count;
    }
    else
    {
        // do not index past command buffer
        if (echo_on) printf_P(PSTR("{\"err\": \"ArgIndxPastCmdBuf\"}\r\n"));
        initCommandBuffer();
        return 0;
    }
}


// white space is not allowed befor the command.
// command always starts at postion 2 and ends at the first white space
// the combined address  and command looks like an MQTT topic or the directory structure of a file system 
// e.g. /0/pwm 127
// find end of command and place a null termination so it can be used as a string
uint8_t findCommand(void) 
{
    uint8_t lastAlpha =2; 
    // if command_buf has "/1/i1scan?", 
    // then command_buf[0] is '/' and command_buf[1] is '1', they are used for addressing
    
    // the command always starts after the addrss at position 2
    command = command_buf + lastAlpha;
    
    // Only an isspace or null may terminate a valid command.
    // The commands first command_buf[2] is '/', then isalpha, followed by isalnum or '?'.
    while( !( isspace(command_buf[lastAlpha]) || (command_buf[lastAlpha] == '\0') ) && lastAlpha < (COMMAND_BUFFER_SIZE-1) ) 
    {
        if ( (lastAlpha == 2) && (command_buf[lastAlpha] == '/') ) // index 0 is '/'
        {
            lastAlpha++;
        }
        else if ( (lastAlpha == 3) && isalpha(command_buf[lastAlpha]) ) // index 1 must be isalpha
        {
            lastAlpha++;
        }
        else if ( (lastAlpha > 3) && ( isalnum(command_buf[lastAlpha]) || (command_buf[lastAlpha] == '?') ) ) 
        {
            lastAlpha++;
        }
        else
        {
            if (echo_on) printf_P(PSTR("{\"err\": \"BadCharInCmd '%c'\"}\r\n"),command_buf[lastAlpha]);
            initCommandBuffer();
            return 0;
        }
    }
    
    // command does  not fit in buffer
    if ( lastAlpha >= (COMMAND_BUFFER_SIZE-1) ) 
    {
        if (echo_on) printf_P(PSTR("{\"err\": \"HugeCmd\"}\r\n"));
        initCommandBuffer();
        return 0;
    }

    if ( isspace(command_buf[lastAlpha]) )
    {
        // the next poistion may be an argument.
        if ( findArgument(lastAlpha+1) )
        {
            // replace the space with a null so command works as a null terminated string.
            command_buf[lastAlpha] = '\0';
        }
        else
        {
            // isspace() found after command but argument was not valid 
            if (echo_on) printf_P(PSTR("{\"err\": \"CharAftrCmdBad '%c'\"}\r\n"),command_buf[lastAlpha+1]);
            initCommandBuffer();
            return 0;
        }
    }
    else
    {
        if (command_buf[lastAlpha] != '\0')
        {
            // null must end command. 
            if (echo_on) printf_P(PSTR("{\"err\": \"MissNullAftrCmd '%c'\"}\r\n"),command_buf[lastAlpha]);
            initCommandBuffer();
            return 0;
        }
    }
    // zero indexing is also the count and should match with strlen()
    return lastAlpha;
}

unsigned long is_arg_in_ul_range (uint8_t arg_num, unsigned long min, unsigned long max)
{
    // check that arg[arg_num] is a digit 
    if ( ( !( isdigit(arg[arg_num][0]) ) ) )
    {
        printf_P(PSTR("{\"err\":\"%sArg%d_NaN\"}\r\n"),command[1],arg_num);
        return 0;
    }
    unsigned long ul = strtoul(arg[arg_num], (char **)NULL, 10);
    if ( ( ul < min) || (ul > max) )
    {
        printf_P(PSTR("{\"err\":\"%sArg%d_OutOfRng\"}\r\n"),command[1],arg_num);
        return 0;
    }
    return ul;
}

// return arg[arg_num] value if in range
uint8_t is_arg_in_uint8_range(uint8_t arg_num, uint8_t min, uint8_t max)
{
    // check that arg[arg_num] is a digit 
    if ( ( !( isdigit(arg[arg_num][0]) ) ) )
    {
        printf_P(PSTR("{\"err\":\"%sArg%d_NaN\"}\r\n"),command[1],arg_num);
        return 0;
    }
    uint8_t argument = atoi(arg[arg_num]);
    if ( ( argument < min) || (argument > max) )
    {
        printf_P(PSTR("{\"err\":\"%sArg%d_OutOfRng\"}\r\n"),command[1],arg_num);
        return 0;
    }
    return argument;
}

