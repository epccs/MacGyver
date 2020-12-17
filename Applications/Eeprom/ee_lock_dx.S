/* This is in the Public Domain
    https://en.wikipedia.org/wiki/Public_domain

    The code is from (Thanks @curtvm)
    https://www.avrfreaks.net/forum/avr128dx-eeprom-writing#comment-3053401
    The compile of this file (ee_lock_dx.S) is not tested but kept for reference.
    
    avr-gcc -c ee_lock_dx.S
    avr-ar -q libavr128da28.a ee_lock_dx.o

    In theory, you can add the lock functions by replacing libavr128da28.a with this new version (or whichever Dx MCU in use)
    */

#ifndef	__DOXYGEN

#include <avr/io.h>

/* Dx and TBD chips have an NVM system with a lock technique to do single-byte erase, read, and write. */
#if E2END && __AVR_XMEGA__ && defined(E2PAGESIZE) && (E2PAGESIZE == 1)

.global eeLock
.global eeUnlock

eeUnlock:
ldi    r25, 0x13 ;EEERWR
rjmp 1f

eeLock:
ldi    r25, 0 ;NOCMD
1:
lds    r24, 0x1400 ;wait on busy
ldi    r24, 0x9D   ;SPM unlock
out    0x34, r24   ;CCP
sts    0x1000, r25 ;cmd
ret

#endif	/* E2END && __AVR_XMEGA__ && defined(E2PAGESIZE) && (E2PAGESIZE = 1) */
#endif	/* !__DOXYGEN__ */
