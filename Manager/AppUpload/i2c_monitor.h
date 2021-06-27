#pragma once

typedef bool (*streamTx_available)(void);

extern void i2c_monitor_init(FILE *debug_uart_to_use, streamTx_available available);
extern uint8_t *got_twi0(void);
extern uint8_t *got_twi1(void);
extern void i2c_monitor(void);