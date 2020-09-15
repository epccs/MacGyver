#ifndef UART0_H
#define UART0_H

// https://www.microchip.com/webdoc/AVRLibcReferenceManual/group__avr__stdio.html
#include <stdio.h>

// Buffer size: (1<<5), (1<<4), (1<<3), (1<<2).
#define UART0_RX0_SIZE (1<<5)
#define UART0_TX0_SIZE (1<<5)

// options
#define UART0_TX_REPLACE_NL_WITH_CR 0x01         // replace transmited newline with carriage return
#define UART0_RX_REPLACE_CR_WITH_NL 0x02         // replace receive carriage return with newline

// error codes
#define UART0_NO_DATA               (1<<0)       // no receive data available bit 0
#define UART0_BUFFER_OVERFLOW       (1<<1)       // receive ringbuffer overflow bit 1
#define UART0_OVERRUN_ERROR         (1<<DOR)     // from USARTn Control and Status Register A bit 3 for Data OverRun (DOR)
#define UART0_FRAME_ERROR           (1<<FE)      // from USARTn Control and Status Register A bit 4 for Frame Error (FE)

// error codes UART_FRAME_ERROR, UART_OVERRUN_ERROR, UART_BUFFER_OVERFLOW, UART_NO_DATA
extern volatile uint8_t UART0_error;

extern void uart0_flush(void);
extern void uart0_empty(void);
extern int uart0_available(void);
extern bool uart0_availableForWrite(void);
extern FILE *uart0_init(uint32_t baudrate, uint8_t choices);
extern int uart0_putchar(char c, FILE *stream);
extern int uart0_getchar(FILE *stream);

#endif // UART0_H 
