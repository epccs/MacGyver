# Uart0_hello is used to kick the tires
TARGET = Uart0_hello
LIBDIR = ../lib
OBJECTS = main.o 

# Chip and project-specific global definitions
MCU   =  avr128da28
F_CPU = 2000000UL/6
#BAUD  =  38400UL
#CPPFLAGS = -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -I. 

# Cross-compilation
# SIDELOAD_TOOLCHAIN = ~/Samba/avr8-10.0.0.2020jan14
#CC = ~/Samba/avr8-10.0.0.2020jan14/gcc-10-strip/bin/avr-gcc
#OBJCOPY = ~/Samba/avr8-10.0.0.2020jan14/gcc-10-strip/bin/avr-objcopy
#OBJDUMP = ~/Samba/avr8-10.0.0.2020jan14/gcc-10-strip/bin/avr-objdump
#SIZE = ~/Samba/avr8-10.0.0.2020jan14/gcc-10-strip/bin/avr-size
# if not SIDELOAD_TOOLCHAIN
CC = avr-gcc
OBJCOPY = avr-objcopy
OBJDUMP = avr-objdump
SIZE = avr-size

# UPDI is the programing interface, it is half-duplex UART based
# Most USB to serial bridges show as /dev/ttyUSB0, 
# Uno's serial bridge (an ATmega16U2) shows as /dev/ttyACM0  (a modem,?)
# Pi Zero on chip hardware serial shows as /dev/ttyAMA0 (hardware UART on a Linux system)
detect_PORT := $(shell sh -c 'ls /dev/ttyAMA0 2>/dev/null || echo not')
ifeq ($(detect_PORT),/dev/ttyAMA0)
	UPDI_PORT = /dev/ttyAMA0
endif
detect_PORT := $(shell sh -c 'ls /dev/ttyUSB0 2>/dev/null || echo not')
ifeq ($(detect_PORT),/dev/ttyUSB0)
	UPDI_PORT = /dev/ttyUSB0
endif

# Compiler/linker options
CFLAGS = -Os -g -std=gnu99 -Wall
# CFLAGS += -funsigned-char -funsigned-bitfields 
# CFLAGS += -fpack-struct -fshort-enums 
CFLAGS += -ffunction-sections -fdata-sections 

# atmega4809 is not in the avr-gcc packaged for my OS 
TARGET_ARCH = -mmcu=$(MCU) \
-B $(LIBDIR)/AVR-Dx_DFP/gcc/dev/avr128da28/ \
-I $(LIBDIR)/AVR-Dx_DFP/include/
## if someday it is in mainline use
##TARGET_ARCH = -mmcu=$(MCU)

LDFLAGS = -Wl,-Map,$(TARGET).map 
LDFLAGS += -Wl,--gc-sections 

.PHONY: help

# some help for the make impaired
# https://marmelab.com/blog/2016/02/29/auto-documented-makefile.html
help:
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

all: $(TARGET).hex $(TARGET).lst ## build the image and its related files

$(TARGET): $(TARGET).hex

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

# # This part has a built in uploader as part of UPDI, so no bootloader is needed but avrdude is not set up to upload to this interface so...
# sudo apt install python3-pip
# pip3 install serial
# # place pyupdi next to your project repository
# git clone https://github.com/mraardvark/pyupdi
# # fuses, what, what?
updi: $(TARGET).hex ## upload with pyupid 
	shell sh -c 'which python3 2>/dev/null || echo install python3'
	shell sh -c 'ls ../../pyupdi/pyupdi.py 2>/dev/null || echo clone pyupdi'
	python3 ../../pyupdi/pyupdi.py -v -d $(MCU) -c $(UPDI_PORT) -b 115200 -e -f $(TARGET).hex

$(TARGET).elf: $(OBJECTS)
	$(CC) $(LDFLAGS) $(TARGET_ARCH) $^ -o $@
#	$(SIZE) -C --mcu=$(MCU) $@
# avr-size -C is not in mainline, it uses pramaters from a config file that needs added for each new chip
# but the pramaters that allow showing % size are now in the elf, 
# avr-objdump -s -j .note.gnu.avr.deviceinfo Uart0_hello.elf
# https://www.eit.lth.se/fileadmin/eit/courses/edi021/Avr-libc-2.0.0/mem_sections.html#sec_dot_note
	$(SIZE) $@
	rm -f $(TARGET).o $(OBJECTS)

clean: ## remove the image and its related files
	rm -f $(TARGET).hex $(TARGET).map $(TARGET).elf $(TARGET).lst
 
%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@
