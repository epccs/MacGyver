/* This is in the Public Domain
    https://en.wikipedia.org/wiki/Public_domain

    The code is from (Thanks @curtvm)
    https://www.avrfreaks.net/forum/avr128dx-eeprom-writing#comment-3053401
    
    example of how to use:
    
    int main(void) 
    {
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
#define EEdolock(statement)             do { eeLock(false); statement eeLock(true); } while(0)

//read/write a type at a 0 based address
#define EEwrite(typ, addr, v)       (*(volatile typ*)(addr+0x1400)) = v
#define EEread(typ, addr)          (*(volatile typ*)(addr+0x1400))

//eeprom macros (with 0 based address)
#define EE_DX_RD_BYTE(addr)      EEread(uint8_t, addr)
#define EE_DX_RD_WORD(addr)      EEread(uint16_t, addr)
#define EE_DX_RD_DWORD(addr)      EEread(uint32_t, addr)
#define EE_DX_WRT_BYTE(addr,v)   EEdolock( EEwrite(uint8_t, addr, v); )
#define EE_DX_WRT_WORD(addr,v)   EEdolock( EEwrite(uint16_t, addr, v); )
#define EE_DX_WRT_DWORD(addr,v)   EEdolock( EEwrite(uint32_t, addr, v); )

// some eeprom.h-like functions, a.k.a. how to use the macros

// Read a byte from EEPROM address
uint8_t eeprom_read_byte (const uint8_t *__p);

// Read a word (16-bit) from EEPROM address
uint16_t eeprom_read_word (const uint16_t *__p);

// Read a double word (32-bit) from EEPROM address
uint32_t eeprom_read_dword (const uint32_t *__p);

// Write a byte to EEPROM address
void eeprom_write_byte (uint8_t *__p, uint8_t __value);

// Write a word (16-bit) to EEPROM address
void eeprom_write_word (uint16_t *__p, uint16_t __value);

// Write a double word (32-bit) to EEPROM address
void eeprom_write_dword (uint32_t *__p, uint32_t __value);

#endif  /* E2END && __AVR_XMEGA__ && defined(E2PAGESIZE) && (E2PAGESIZE = 1) */
#endif  /* !__DOXYGEN__ */

