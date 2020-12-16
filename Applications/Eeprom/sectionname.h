#ifndef __SECTIONNAME_H__
#define __SECTIONNAME_H__

/* avr-libc functions are put in common, unique sub-section name under .text. */

#define CLIB_SECTION    .text.avr-libc
#define MLIB_SECTION    .text.avr-libc.fplib

#define STR(x)   _STR(x)
#define _STR(x)  #x

#define ATTRIBUTE_CLIB_SECTION  __attribute__ ((section (STR(CLIB_SECTION))))
#define ATTRIBUTE_MLIB_SECTION  __attribute__ ((section (STR(MLIB_SECTION))))

#define ASSEMBLY_CLIB_SECTION   .section CLIB_SECTION, "ax", @progbits
#define ASSEMBLY_MLIB_SECTION   .section MLIB_SECTION, "ax", @progbits

#endif

