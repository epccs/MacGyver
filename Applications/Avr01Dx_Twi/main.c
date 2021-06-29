#include <stdbool.h>
#include <stdint.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>
#include "../lib/timers_bsd.h"
#include "../lib/uart0_bsd.h"
#include "twis.h"
#include "twim.h"

FILE *uart0;

#if 0
#include "ds3231.h"
/*------------------------------------------------------------------------------
    slave - address 0x44, respond to read command 0x55 with v value
------------------------------------------------------------------------------*/
bool twisCallback(twis_irqstate_t state, uint8_t statusReg){
    bool ret = true;
    static uint8_t v;

    switch( state ) {
        case TWIS_ADDRESSED: //1
            ret = twis_lastAddress() == 0x44; //for us?
            break;
        case TWIS_MREAD: //3
            twis_write( v++ ); //respond
            break;
        case TWIS_MWRITE: //2
            ret = (twis_read() == 0x55); //valid command?
            break;
        case TWIS_STOPPED:
        case TWIS_ERROR:
            ret = false;
            break;
        }
    return ret;
}

//send command to slave
void testSlave(){
    twis_defaultPins();             //slave pins
    twis_init( 0x44, twisCallback );//0x44, callback function above

    twim_defaultPins();             //master pins (same as slave)
    twim_baud( F_CPU, 100000ul );   //100kHz
    twim_on( 0x44 );                //on, slave address 0x44
    uint8_t wrbuf[1] = { 0x55 };         //command
    uint8_t rdbuf[1];                    //read 1 byte
    twim_writeRead( wrbuf, 1, rdbuf, 1 ); //do transaction, 1 write, 1 read
    twim_waitUS( 3000 );            //wait for complettion or timeout (3ms)
}


//watch w/logic analyzer
int main(){

    while(1){

        testSlave();
        _delay_ms(10);

        uint8_t sec;
        ds3231_seconds( &sec );
        _delay_ms(1000);

        }

}
#endif

#if 1
/*------------------------------------------------------------------------------
    slave - address 0x51
------------------------------------------------------------------------------*/
bool twisCallback(twis_irqstate_t state, uint8_t statusReg){
    bool ret = true;

    switch( state ) {
        case TWIS_ADDRESSED:
            ret = (twis_lastAddress() == 0x51); //for us?
            break;
        case TWIS_MREAD:
            twis_write( statusReg ); //respond (just send status register value)
            break;
        case TWIS_MWRITE:
        case TWIS_STOPPED:
        case TWIS_ERROR:
            ret = false;
            break;
        }
    return ret;
}

#define TWI_TTL 3000UL
#define TWI_DELAY 5UL
unsigned long twi_started_at;
unsigned long twi_started_at_safe;
unsigned long twi_ttl;
unsigned long twi_delay;

uint16_t twim_cb_timout_count; // how many times did the twim_cb timeout hit
uint16_t twim_cb_good_count;
uint16_t twim_cb_bad_count;
unsigned long twim_cb_last_elapsed;
uint16_t twim_cb_elapsed_lessthan_delay;
volatile bool twim_cb_interlock;
uint8_t rdbuf[5];

uint16_t cp_twim_cb_timout_count; // a copy for printing
uint16_t cp_twim_cb_good_count;
uint16_t cp_twim_cb_bad_count;
unsigned long cp_twim_cb_last_elapsed;
uint16_t cp_twim_cb_elapsed_lessthan_delay;
uint8_t print_part; // print the data in parts so uart does not block

/*
void abort_safe(void)
{
    // flush the UART befor halt
    uart0_flush();
    twim_off(); // need to clear the pins
    _delay_ms(20); // wait for last byte to send
    uart0_init(0, 0); // disable UART hardware
    // turn off interrupts and then spin loop
    cli();
    while(1)
    {
        _delay_ms(100);
    }
}
*/

// finish a twi transaction that was started but is now done
void twimCallback(void) {
    if (twim_lastResultOK()) {
        ++twim_cb_good_count;
    } else {
        ++twim_cb_bad_count;
    }
    twim_cb_last_elapsed = elapsed(&twi_started_at);
    if (twim_cb_last_elapsed < twi_delay) {
        ++twim_cb_elapsed_lessthan_delay; // counts when interlock is done wrong
    } else { // timing will break if twi_started_at gets ahead of tickAtomic()
        twi_started_at += twi_delay;
    }
    twim_cb_interlock = false;
    twim_callback(NULL); // remove the callback, the ISR checks for NULL and this will cause it to be ignored.
}

//send command to slave
void testSlave() {
    // the callback is run in ISR context and will update the four bytes of 
    // twi_started_at (while the main thread is in the middle of using them).
    if (twim_cb_interlock) { // this interlock tells if twi_started_at is safe to use
        unsigned long kRuntime = elapsed(&twi_started_at_safe); // this time is safe when isr is active
        if ( kRuntime > twi_ttl) 
        {
            // timed out
            ++twim_cb_timout_count;
            twim_cb_interlock = false;
            twim_off();
            twim_callback(NULL);
            fprintf_P(uart0,PSTR("twi0 timed out\r\n"));
            while(!uart0_availableForWrite()); // block until uart is done
            twi_started_at = tickAtomic(); // restart the timer.
         }
    } else {
        unsigned long kRuntime = elapsed(&twi_started_at); // time is safe since interlock shows ISR will not run
        if (!twim_isBusy()) { // twi not busy
            if ((kRuntime > twi_delay)) { // the delay for next transaction has passed
                twim_callback(twimCallback); // register what to do after transaction has finished
                twim_on( 0x51 ); //on, slave address 0x51
                twim_read( rdbuf, sizeof(rdbuf) ); //do transaction, read n bytes
                twim_cb_interlock = true;
                twi_started_at_safe = twi_started_at; // used for ttl timing, the ISR will not change it
            }
        }
    }
    if (uart0_availableForWrite()) { // if UART is done then print latest twi stats (but don't block).
        if (print_part == 0) {
            cli(); // get a copy that will not change
            cp_twim_cb_last_elapsed = twim_cb_last_elapsed;
            cp_twim_cb_timout_count = twim_cb_timout_count;
            cp_twim_cb_good_count = twim_cb_good_count;
            cp_twim_cb_bad_count = twim_cb_bad_count;
            cp_twim_cb_elapsed_lessthan_delay = twim_cb_elapsed_lessthan_delay;
            sei();
            fprintf_P(uart0,PSTR("ticks %lu, "), cp_twim_cb_last_elapsed);
            print_part = 1;
        } else if (print_part == 1) {
            fprintf_P(uart0,PSTR("tmo %u, "),  cp_twim_cb_timout_count);
            print_part = 2;
        } else if (print_part == 2) {
            fprintf_P(uart0,PSTR("short %u, "),  cp_twim_cb_elapsed_lessthan_delay);
            print_part = 3;
        } else if (print_part == 3) {
            fprintf_P(uart0,PSTR("good %u, "), cp_twim_cb_good_count);
            print_part = 4;
        } else if (print_part == 4) {
            fprintf_P(uart0,PSTR("bad %u\r\n"), cp_twim_cb_bad_count);
            print_part = 0;
        } else { // this should not happen
            print_part = 0;
        }
        
    }
}


//watch w/logic analyzer
int main() {

    // timers will take F_CPU from Makefile and set the clock (16MHz)
    initTimers(); //TCA0_HUNF used for timing, TCA0 is split for 6 PWM's on WO0..WO5.

    /* Initialize UART0 to 38.4kbps for streaming, it returns a pointer to a FILE structure */
    uart0 = uart0_init(38400UL, UART0_RX_REPLACE_CR_WITH_NL);

    twim_defaultPins();             //master pins (same as slave)
    twim_baud( F_CPU, 100000ul );   //100kHz

    twis_defaultPins();             //slave pins
    twis_init( 0x51, twisCallback );//0x51, callback function above

    twi_delay = cnvrt_milli(TWI_DELAY);
    twi_ttl = cnvrt_milli(TWI_TTL);

    sei();

    // loop
    while(1) {
        testSlave();
    }

}
#endif