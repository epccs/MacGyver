#pragma once

// https://www.microchip.com/webdoc/AVRLibcReferenceManual/group__avr__stdio.html
#include <stdio.h>

// Buffer size: (1<<5), (1<<4), (1<<3), (1<<2).
#define RX1_SIZE (1<<5)
#define TX1_SIZE (1<<5)

// options
#define UART1_TX_REPLACE_NL_WITH_CR 0x01         // replace transmited newline with carriage return
#define UART1_RX_REPLACE_CR_WITH_NL 0x02         // replace receive carriage return with newline

// error codes
#define UART1_NO_DATA               (1<<0)       // no receive data available bit 0
#define UART1_BUFFER_OVERFLOW       (1<<1)       // receive ringbuffer overflow bit 1
#define UART1_OVERRUN_ERROR         (1<<DOR)     // from USARTn Control and Status Register A bit 3 for Data OverRun (DOR)
#define UART1_FRAME_ERROR           (1<<FE)      // from USARTn Control and Status Register A bit 4 for Frame Error (FE)

// error codes UART_FRAME_ERROR, UART_OVERRUN_ERROR, UART_BUFFER_OVERFLOW, UART_NO_DATA
extern volatile uint8_t UART1_error;

extern void uart1_flush(void);
extern void uart1_empty(void);
extern int uart1_available(void);
extern bool uart1_availableForWrite(void);
extern FILE *uart1_init(uint32_t baudrate, uint8_t choices);
extern int uart1_putchar(char c, FILE *stream);
extern int uart1_getchar(FILE *stream);
