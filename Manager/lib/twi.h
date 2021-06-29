#pragma once

/*------------------------------------------------------------------------------
    twim - Twi0, master - avr mega0, tiny0/1, da

    1. select pins to use- default or alternate
        twim_defaultPins(); //pullups on, portmux set
    2. set baud
        twim_baud( F_CPU, 100000ul ); //100kHz
    3. turn on, specifying slave address
        twim_on(0x44);
    4. enable interrupts via sei() (avr/interrupts.h)


    optionally set a callback function if not polling via twim_isBusy() or
    twim_waitUS()
        twim_callback(myCallback);

    you can now use one of four functions to move data-

    twim_writeRead - write wbuffer of wN length, read into rbuffer of rN length
    twim_writeWrite - write wbuffer of wN length, write wbuffer2 of w2N length
    twim_write - write wbuffer of wN length (alias to writeRead with no read)
    twim_read - read into rbuffer of rN length (alias to writeRead with no write)

    if not using a callback function, you can poll for completion-

        uint8_t wbuf[1] = { 0x55 }; //command to read 4 bytes, as an example
        uint8_t rbuf[4]; //no need to clear/init
        twim_writeRead( wbuf, 1, rbuf, 4 );

        //blocking until done
        while( twim_isBusy() ){} //blocks until done
        if( twim_lastResultOK() ) //rbuf has 4 bytes
        else //was nack'd or bus error/collision

        //or use a timeout in us
        if( twim_waitUS(3000) ) //rbuf has 4 bytes
        else if( twim_isBusy() ) //was timeout, (twim irqs may still be on)
        else //was nack'd or bus error/collision (twim irqs are off)

        twim_off();


------------------------------------------------------------------------------*/

typedef void (*twim_callbackT)(void);

void twim_off               ();
void twim_on                (uint8_t address);
bool twim_isBusy            ();
bool twim_lastResultOK      ();
void twim_callback          (twim_callbackT callbackFunction);
void twim_writeRead         (const uint8_t* writeBuffer, uint16_t writeLength, uint8_t* readBuffer, uint16_t readLength);
void twim_writeWrite        (const uint8_t* writeBuffer, uint16_t writeLength, const uint8_t* writeBuffer2, uint16_t writeLength2);
void twim_write             (const uint8_t* writeBuffer, uint16_t writeLength);
void twim_read              (uint8_t* readBuffer, uint16_t readLength);
bool twim_waitUS            (uint16_t microseconds);
void twim_defaultPins       ();
void twim_altPins           ();

                            __attribute((always_inline)) inline static
void twim_baud              (uint32_t cpuHz, uint32_t twiHz)
                            {
                            int32_t v = cpuHz/twiHz/2 - 5;
                            TWI0.MBAUD = v >= 0 ? v : 0;
                            }

/*------------------------------------------------------------------------------
    twis - Twi slave

    1. select pins to use- default or alternate
    twis_defaultPins();
    2. init with address and callback function
    twis_init( 0x40, myCallback);
    3. enable interrupts via sei() (avr/interrupts.h)

    optional - set a 2nd address, or an address mask, can be set at anytime


    callback function, isr provides state and status register
    (status register not really necessary, but can be used get more info on
     when in TWIS_ERROR state)

    bool myCalback(twis_irqstate_t state, uint8_t statusReg){
        you have the enum states to deal with as needed
        return true if everything ok, false if you want to stop the transaction
        you have 3 functions to use with twis-
            twis_lastAddress - when in TWIS_ADDRESSED, this will give address
                               seen to get here (could be 0, address, address2,
                               or a mask match)
            twis_write - when in TWIS_MREAD, you can reply with a write
            twis_read - when in TWIS_MWRITE, you can read what was sent
    }

    NOTE- gencall is enabled by default, so check the address in the callback
          when in TWIS_ADDRESSED state (simply enabled to eliminate one more
          option to set- most likely never seen but if so you can ignore in the
          callback by only returning true when dealing with an address you
          want to respond to)
------------------------------------------------------------------------------*/

typedef enum { TWIS_ADDRESSED, TWIS_MREAD, TWIS_MWRITE, TWIS_STOPPED,
               TWIS_ERROR } twis_irqstate_t;
typedef bool(*twis_callback_t)(twis_irqstate_t state, uint8_t statusReg);

void twis_off               ();
void twis_write             (uint8_t value);
uint8_t twis_read           ();
uint8_t twis_lastAddress    ();
void twis_address2          (uint8_t SlaveAddress2);
void twis_addressMask       (uint8_t SlaveAddressMask); //no 2nd address
void twis_init              (uint8_t SlaveAddress, twis_callback_t callbackFunction);
void twis_defaultPins       ();
void twis_altPins           ();


#if defined(TWI1)
void twi1m_off              ();
void twi1m_on               (uint8_t address);
bool twi1m_isBusy           ();
bool twi1m_lastResultOK     ();
void twi1m_callback         (twim_callbackT callbackFunction);
void twi1m_writeRead        (const uint8_t* writeBuffer, uint16_t writeLength, uint8_t* readBuffer, uint16_t readLength);
void twi1m_writeWrite       (const uint8_t* writeBuffer, uint16_t writeLength, const uint8_t* writeBuffer2, uint16_t writeLength2);
void twi1m_write            (const uint8_t* writeBuffer, uint16_t writeLength);
void twi1m_read             (uint8_t* readBuffer, uint16_t readLength);
bool twi1m_waitUS           (uint16_t microseconds);
void twi1m_defaultPins      ();
void twi1m_altPins          ();

                            __attribute((always_inline)) inline static
void twi1m_baud             (uint32_t cpuHz, uint32_t twiHz)
                            {
                            int32_t v = cpuHz/twiHz/2 - 5;
                            TWI1.MBAUD = v >= 0 ? v : 0;
                            }
void twi1s_off              ();
void twi1s_write            (uint8_t value);
uint8_t twi1s_read          ();
uint8_t twi1s_lastAddress   ();
void twi1s_address2         (uint8_t SlaveAddress2);
void twi1s_addressMask      (uint8_t SlaveAddressMask); //no 2nd address
void twi1s_init             (uint8_t SlaveAddress, twis_callback_t callbackFunction);
void twi1s_defaultPins      ();
void twi1s_altPins          ();
#endif