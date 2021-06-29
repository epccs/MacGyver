//======================================================================
//  twim.c - Twi0, master - avr mega0, tiny0/1, da
//======================================================================
#include <stdbool.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <avr/io.h>
#include <util/delay.h>

#include "twi1m.h"
#include "twiPins.h"

    //==========
    // private:
    //==========

static twim_callbackT   isrFuncCallback_;
static const uint8_t*        txbuf_;
static const uint8_t*        txbufEnd_;
static const uint8_t*        txbuf2_;
static const uint8_t*        txbuf2End_;
static uint8_t*              rxbuf_;
static const uint8_t*        rxbufEnd_;
static volatile bool    lastResult_; //1=ok,0=fail

//local enums
//MCTRLB flush3|ack2|cmd1:0
enum { ACK = 0, READ = 2, STOP = 3, NACK = 4,  FLUSH = 8 };
//MSTATUS RIF7|WIF6|CLKHOLD5|RXACK4|ARBLOST3|BUSERR2|BUSSTATE1:0
enum { RIF = 0x80, WIF = 0x40, CLKHOLD = 0x20, RXNACK = 0x10, ARBLOST = 0x8, BUSERR = 0x4 };
enum { ALLFLAGS = RIF|WIF|CLKHOLD|ARBLOST|BUSERR };
enum { ANYERR = ARBLOST|BUSERR }; //error bits
enum { RIEN = RIF, WIEN = WIF, RWIEN = RIEN|WIEN }; //irq bits
enum { RW = 1 }; //address bit0
enum { UNKNOWN = 0, IDLE, OWNER, BUSBUSY, BUSMASK = 3 }; //bus state
enum { READOK = RIF|CLKHOLD|OWNER, WRITEOK = WIF|CLKHOLD|OWNER };
enum { ENABLE = 1 }; //on/off

static void on              () { TWI1.MCTRLA |= ENABLE; }
static void off             () { TWI1.MCTRLA = 0; }
static void irqAllOn        () { TWI1.MCTRLA |=  RWIEN; }
static void irqAllOff       () { TWI1.MCTRLA &= ~RWIEN; }
static void toStateIdle     () { TWI1.MSTATUS = ALLFLAGS|IDLE; } //clear flags, set to IDLE
static void ackActionACK    () { TWI1.MCTRLB = ACK; }
static void ACKread         () { TWI1.MCTRLB = READ; }
static void NACKstop        () { TWI1.MCTRLB = NACK|STOP; }
static void address         (uint8_t v) { off(); TWI1.MADDR = v<<1; } //off so no start produced
static void startRead       () { ackActionACK(); TWI1.MADDR |= RW; } //reuse existing address
static void startWrite      () { TWI1.MADDR &= ~RW; }   //reuse existing address
static void write           (uint8_t v) { TWI1.MDATA = v; }
static uint8_t   read            () { return TWI1.MDATA; }
static uint8_t   status          () { return TWI1.MSTATUS; }
static bool isBusy          () { return TWI1.MCTRLA & RWIEN; }

                            //start a read or write, enable irq
static void startIrq        (bool wr)
                            {
                            if( wr ) startWrite(); else startRead();
                            lastResult_ = false;
                            irqAllOn();
                            }

                            //for isr use
static void finished        (bool tf)
                            {
                            lastResult_ = tf;
                            //NACKstop works for write also (nack not done, harmless)
                            NACKstop();
                            irqAllOff(); //do before callback in case call back starts another xfer
                            if( isrFuncCallback_ ) isrFuncCallback_();
                            }


ISR(TWI1_TWIM_vect)         {
                            uint8_t s = status();
                            //error
                            if( s & ANYERR ) return finished( false );
                            //read
                            if( s == READOK ){
                                *rxbuf_++ = read();
                                return rxbuf_ < rxbufEnd_ ? ACKread() : finished( true );
                                }
                            //write
                            if( s == WRITEOK ){
                                if( txbuf_ < txbufEnd_ ) return write( *txbuf_++ ); //more data
                                if( txbuf2_ < txbuf2End_ ) return write( *txbuf2_++ ); //more data
                                return rxbuf_ ? startRead() : finished( true ); //switch to read? or done
                                }
                            //unknown, or a write nack
                            finished( false );
                            }

    //==========
    // public:
    //==========

void    twim_callback       (twim_callbackT cb) { isrFuncCallback_ = cb; } //optional, else use twim_waitUS
void    twim_off            () { off(); }
void    twim_on             (uint8_t addr) { address(addr); toStateIdle(); on(); }
bool    twim_isBusy         () { return isBusy(); } //if irq on, is busy
bool    twim_lastResultOK   () { return lastResult_; }

                            //set default or alternate pins
void    twim_defaultPins    () { TWI_PULL_DEFAULT(); TWI_PORTMUX_DEFAULT(); }
void    twim_altPins        () { TWI_PULL_ALT(); TWI_PORTMUX_ALT(); }


                            //write+read (or write only, or read only)
void    twim_writeRead      (const uint8_t* wbuf, uint16_t wn, uint8_t* rbuf, uint16_t rn)
                            {
                            txbuf_ = wbuf; txbufEnd_ = &wbuf[wn];
                            rxbuf_ = rbuf; rxbufEnd_ = &rbuf[rn];
                            txbuf2_ = 0; txbuf2End_ = 0;
                            startIrq( wn ); //if no write (wn==0), then will start a read irq
                            }

                            //write/write (such as a command, then a buffer)
void    twim_writeWrite     (const uint8_t* wbuf, uint16_t wn, const uint8_t* wbuf2, uint16_t wn2)
                            {
                            txbuf_ = wbuf; txbufEnd_ = &wbuf[wn];
                            txbuf2_ = wbuf2; txbuf2End_ = &wbuf2[wn2];
                            rxbuf_ = 0; rxbufEnd_ = 0; //no read
                            startIrq( 1 ); //write only
                            }

                            //write only alias
void    twim_write          (const uint8_t* wbuf, uint16_t wn) { twim_writeRead( wbuf, wn, 0, 0); }

                            //read only alias
void    twim_read           (uint8_t* rbuf, uint16_t rn) { twim_writeRead( 0, 0, rbuf, rn); }

                            //blocking wait with timeout
bool    twim_waitUS         (uint16_t us)
                            {
                            while( _delay_us(1), --us && twim_isBusy() ){}
                            return twim_lastResultOK();
                            //lastResult_ is set to false at start of transaction
                            //will still be false if timeout
                            //check twim_isBusy() on your own to see if was a timeout (if returned false)
                            }

