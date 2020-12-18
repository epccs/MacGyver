#ifndef ee_H
#define ee_H

// eeprom size is in ioavr128da28.h
#ifndef EEPROM_SIZE
#   error your mcu is not supported
#endif

extern void EEread_cmd(void);
extern void EEwrite_cmd(void);

#endif // ee_H 
