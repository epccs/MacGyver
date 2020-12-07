#ifndef ee_H
#define ee_H

// eeprom size is in ioavr128da28.h
#ifndef EEPROM_SIZE
#   error your mcu is not supported
#endif

extern void EEread(void);
extern void EEwrite(void);

#endif // ee_H 
