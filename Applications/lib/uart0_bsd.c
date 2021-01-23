/*
Interrupt-Driven UART for AVR Standard IO facilities streams 
Copyright (C) 2020 Ronald Sutherland

Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES 
WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE 
FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY 
DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, 
WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, 
ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

https://en.wikipedia.org/wiki/BSD_licenses#0-clause_license_(%22Zero_Clause_BSD%22)

API is done in C for AVR Standard IO facilities streams
https://www.microchip.com/webdoc/AVRLibcReferenceManual/group__avr__stdio.html

The standard streams stdin, stdout, and stderr are provided, but contrary to the C standard, 
since avr-libc has no knowledge about applicable devices, these streams are not already 
pre-initialized at application startup. Also, since there is no notion of "file" whatsoever to 
avr-libc, there is no function fopen() that could be used to associate a stream to some device. 
Instead, the function fdevopen() is provided to associate a stream to a device, where the device 
needs to provide a function to send a character, to receive a character, or both. There is no 
differentiation between "text" and "binary" streams inside avr-libc. Character \n is sent literally 
down to the device's put() function. If the device requires a carriage return (\r) character to be 
sent before the linefeed, its put() routine must implement this 

UART0_TX_REPLACE_NL_WITH_CR and UART0_RX_REPLACE_CR_WITH_NL may be used 
to filter data into and out of the uart.

Getting Started with USART: https://github.com/microchip-pic-avr-examples/avr128da48-getting-started-with-usart-mplab-mcc
*/

#include <stdio.h>
#include <stdbool.h>
#include <util/atomic.h>
#include "io_enum_bsd.h"
#include "uart0_bsd.h"

// Asynchronous Normal mode (more error tolerate)
#define UART0_BAUD_SELECT_NS(baudRate) ((float)(F_CPU * 64 / (16 * (float)baudRate)) + 0.5)
//Asynchronous Double-Speed mode
#define UART0_BAUD_SELECT_DS(baudRate) ((float)(F_CPU * 64 / (8 * (float)baudRate)) + 0.5)

static volatile uint8_t TxBuf[UART0_TX0_SIZE];
static volatile uint8_t RxBuf[UART0_RX0_SIZE];
static volatile uint8_t TxHead;
static volatile uint8_t TxTail;
static volatile uint8_t RxHead;
static volatile uint8_t RxTail;

static uint8_t options;
volatile uint8_t UART0_error;

/* Receive Complete interrupt occures for three event conditions
     * There is unread data in the receive buffer (RXCIE)
     * Receive of Start-of-Frame detected (RXSIE)
     * Auto-Baud Error/ISFIF flag set (ABEIE)
*/
ISR(USART0_RXC_vect) // 328p: USART0_RX_vect 
{
    uint16_t next_index;
    uint8_t data;
 
    // check USARTn Control and Status Register A for Frame Error (FERR) or Buffer Overflow (BUFOVF) [Parity Error (PERR) kept but not used]
    // 328p: uint8_t last_status = (UCSR0A & ((1<<FE)|(1<<DOR)) );
    uint8_t last_status = (USART0.RXDATAH  & ((1<<USART_FERR_bp)|(1<<USART_BUFOVF_bp)|(1<<USART_PERR_bp)) );

    // for 8 bit (and less) reading RXDATAL will shift the data buffer (doubled buffered) so read it after RXDATAH.
    data = USART0.RXDATAL;

    next_index = ( RxHead + 1) & ( UART0_RX0_SIZE - 1);
    
    if ( next_index == RxTail ) 
    {
        last_status += UART0_BUFFER_OVERFLOW;
    } 
    else 
    {
        RxHead = next_index;
        RxBuf[next_index] = data;
    }
    UART0_error = last_status;   
}

/* Data Register Empty interrupt occures for one event condition
     * The transmit buffer is empty/ready to receive new data (DREIE)
*/
ISR(USART0_DRE_vect) // 328p: USART0_UDRE_vect
{
    uint16_t tmptail;

    if ( TxHead != TxTail) 
    {
        tmptail = (TxTail + 1) & ( UART0_TX0_SIZE - 1); // calculate and store new buffer index
        TxTail = tmptail;
        USART0.STATUS = USART_TXCIF_bm;
        USART0.TXDATAL = TxBuf[tmptail]; // get one byte from buffer and send it with UART
    } 
    else 
    {
        // Disable the Data Register Empty Interrupt Enable bit since tx buffer empty
        USART0.CTRLA &= (~USART_DREIE_bm);
    }
}

// Flush bytes from the transmit buffer with busy waiting.
void uart0_flush(void)
{
    while (TxHead != TxTail)
    {
        //busy waiting
    };
}

// Immediately stop transmitting by removing any buffered outgoing serial data.
// helps to reduce/avoid collision damage on full-duplex multi-drop
void uart0_empty(void)
{
    TxHead = TxTail;
}

// Number of bytes available in the receive buffer.
int uart0_available(void)
{
    return (UART0_RX0_SIZE + RxHead - RxTail) & ( UART0_RX0_SIZE - 1);
}

// Transmit buffer (all of it) is available for writing without blocking.
bool uart0_availableForWrite(void)
{
    return (TxHead == TxTail);
}

// Protofunctions (code is latter) to allow UART0 to be used as a stream for printf, scanf, etc...
int uart0_putchar(char c, FILE *stream);
int uart0_getchar(FILE *stream);

// Stream declaration for stdio
static FILE uartstream0_f = FDEV_SETUP_STREAM(uart0_putchar, uart0_getchar, _FDEV_SETUP_RW);

// Initialize USART0 and return file handle
// disable UART if baudrate (a.k.a., bitrate) is zero
// choices e.g., UART0_TX_REPLACE_NL_WITH_CR & UART0_RX_REPLACE_CR_WITH_NL
FILE *uart0_init(uint32_t baudrate, uint8_t choices)
{
    // an stomic transaction is done by turning off interrupts
    uint8_t oldSREG = SREG;
    cli();           // clear the global interrupt mask.

    TxHead = 0;
    TxTail = 0;
    RxHead = 0;
    RxTail = 0;

    // disconnect UART if baudrate is zero
    if (baudrate == 0)
    {
        USART0.CTRLB &= ~(USART_RXEN_bm | USART_TXEN_bm); // Disable receiver and transmitter
        USART0.CTRLA &= ~(USART_RXCIE_bm | USART_DREIE_bm); // Disable RX complete and data register empty interrupts
    }
    else
    {   
        USART0.CTRLB &= (~USART_RXMODE_CLK2X_gc);
        USART0.CTRLB |= USART_RXMODE_NORMAL_gc;
        USART0.BAUD = UART0_BAUD_SELECT_NS(baudrate);

        // control frame format asynchronous, 8data, no parity, 1stop bit
        USART0.CTRLC = USART_CMODE_ASYNCHRONOUS_gc | USART_CHSIZE_8BIT_gc | USART_PMODE_DISABLED_gc | USART_SBMODE_1BIT_gc;
 
        // enable TX, RX, and Receive Complete Interrupt
        USART0.CTRLB |= (USART_RXEN_bm | USART_TXEN_bm);
        USART0.CTRLA |= USART_RXCIE_bm;

        // Default pins MCU_IO_RX0:PA1, MCU_IO_TX0:PA0 [or alternative MCU_IO_MISO:PA5, MCU_IO_MOSI:PA4]
        PORTMUX.USARTROUTEA = PORTMUX_USART0_DEFAULT_gc; // [PORTMUX_USART0_ALT1_gc]

        // set RX0 with weak pullup and TX0 as output
        ioDir(MCU_IO_RX0, DIRECTION_INPUT);
        ioCntl(MCU_IO_RX0, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_ENABLE, PORT_INVERT_NORMAL);
        ioDir(MCU_IO_TX0, DIRECTION_OUTPUT);
        ioCntl(MCU_IO_TX0, PORT_ISC_INTDISABLE_gc, PORT_PULLUP_DISABLE, PORT_INVERT_NORMAL);
    }

    options = choices;

    SREG = oldSREG; // restore global interrupt if they were enabled

    return &uartstream0_f;
}

// putchar for sending to stdio stream
int uart0_putchar(char c, FILE *stream)
{
    uint16_t next_index;

    next_index  = (TxHead + 1) & ( UART0_TX0_SIZE - 1);

    while ( next_index == TxTail ) 
    {
        ;// busy wait for free space in buffer
    }

    // I put a carriage return and newline in the printf string  
    // so I don't use UART0_TX_REPLACE_NL_WITH_CR
    if ( (options & UART0_TX_REPLACE_NL_WITH_CR) && (c == '\n') )
    {
        TxBuf[next_index] = (uint8_t)'\r';
    }
    else
    {
        TxBuf[next_index] = (uint8_t) c;
    }
    TxHead = next_index;

    // Enable the Data Register Empty Interrupt Enable bit
    USART0.CTRLA |= USART_DREIE_bm;

    return 0;
}

// getchar for reading from stdio stream
int uart0_getchar(FILE *stream)
{
    uint16_t next_index;
    uint8_t data;

    while( !(uart0_available()) );  // wait for input

    if ( RxHead == RxTail ) 
    {
        UART0_error += UART0_NO_DATA;
        data = 0;
    }
    else
    {
        next_index = (RxTail + 1) & ( UART0_RX0_SIZE - 1);
        RxTail = next_index;
        data = RxBuf[next_index]; // get byte from rx buffer
    }

    // I use UART0_RX_REPLACE_CR_WITH_NL to simplify command parsing from a host 
    if ( (options & UART0_RX_REPLACE_CR_WITH_NL) && (data == '\r') ) data = '\n';
    return (int) data;
}


