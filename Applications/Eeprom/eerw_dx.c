/* This is in the Public Domain
    https://en.wikipedia.org/wiki/Public_domain

    The code is from (Thanks @curtvm)
    https://www.avrfreaks.net/forum/avr128dx-eeprom-writing#comment-3053401
    
    example of how to use:
    
    int main(void) 
    {
        eeprom_write_byte( &ee0, 0x55 );
        eeprom_write_byte( 1, 0xAA );

        //a bypass of the eeprom functions that can do multiple writes with eeprom unlocked
        EEdo( EEwrite( uint16_t, 2, 0x1234 ); EEwrite( uint16_t, 4, 0x5678 );  );
        for(;;){}
    }
    
    
*/

#ifndef __DOXYGEN__

#include <avr/io.h>

/* Dx and TBD chips have an NVM system with a lock technique to do single-byte erase, read, and write. */
#if E2END && __AVR_XMEGA__ && defined(E2PAGESIZE) && (E2PAGESIZE == 1U)
#include <stdint.h>
#include <stdbool.h>

#define EEMEM __attribute(( section(".eeprom") ))

 //lock/unlock eeprom
static inline void eeLock(bool tf){
    *(volatile uint8_t*)0x1400; //ee read blocks until ee ready
    if(tf) _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA,0);
    else   _PROTECTED_WRITE_SPM(NVMCTRL.CTRLA,0x13);
}

// group a set of eeprom writes
#define EEdo(statement)             do { eeLock(false); statement eeLock(true); } while(0)

//read/write a type at a 0 based address
#define EEwrite(typ, addr, v)       (*(volatile typ*)(addr+0x1400)) = v
#define EEread (typ, addr)          (*(volatile typ*)(addr+0x1400))

//standard eeprom functions (0 based address)
#define ee_dx_write_byte(addr,v)   EEdo( EEwrite(uint8_t, addr, v); )
#define ee_dx_read_byte(addr)      EEread (uint8_t, addr, v)

//need to add some more eeprom.h-like functions

//0 based (standard linker script)
EEMEM uint8_t ee0 = 5;

#endif  /* E2END && __AVR_XMEGA__ && defined(E2PAGESIZE) && (E2PAGESIZE = 1) */
#endif  /* !__DOXYGEN__ */

