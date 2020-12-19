/*  This is in the Public Domain
    https://en.wikipedia.org/wiki/Public_domain

    The code is from (Thanks @curtvm)
    https://www.avrfreaks.net/forum/avr128dx-eeprom-writing#comment-3053401
    
    example of how to use:

    int main(void) 
    {
        eeprom_write_byte( 1, 0xAA );

        //a macro bypass of the eeprom functions that does multiple writes with eeprom unlocked
        EEdo( EEwrite( uint16_t, 2, 0x1234 ); EEwrite( uint16_t, 4, 0x5678 );  );
        for(;;){}
    }

avr-libc may have some String tokenization in C 
https://onebyezero.blogspot.com/2018/12/string-tokenization-in-c.html
*/
#include "eerw_dx.h"

//standard eeprom functions (with 0 based address) based somewhat on eeprom.h from avr-libc
uint8_t eeprom_read_byte (const uint8_t *__p)
{
    return EE_DX_RD_BYTE(__p);
}

uint16_t eeprom_read_word (const uint16_t *__p)
{
    return EE_DX_RD_WORD(__p);
}

uint32_t eeprom_read_dword (const uint32_t *__p)
{
    return EE_DX_RD_DWORD(__p);
}

void eeprom_write_byte (uint8_t *__p, uint8_t __value)
{
    EE_DX_WRT_BYTE(__p,__value);
}

void eeprom_write_word (uint16_t *__p, uint16_t __value)
{
    EE_DX_WRT_WORD(__p,__value);
}

void eeprom_write_dword (uint32_t *__p, uint32_t __value)
{
    EE_DX_WRT_DWORD(__p,__value);
}