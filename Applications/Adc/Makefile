# used to kick the tires
TARGET = Adc
LIBDIR = ../lib
OBJECTS = main.o \
	analog.o \
	../Uart/id.o \
	$(LIBDIR)/timers_bsd.o \
	$(LIBDIR)/uart0_bsd.o \
	$(LIBDIR)/twi.o \
	$(LIBDIR)/adc_bsd.o \
	$(LIBDIR)/references.o \
	$(LIBDIR)/rpu_mgr.o \
	$(LIBDIR)/parse.o

# Chip and project-specific global definitions
MCU = avr128da28
# 1,2,4*,8,16. * is default: Internal High-Frequency Oscillator Control A (OSCHFCTRLA) bitfield FRQSEL[3:0]
F_CPU = 16000000UL
CPPFLAGS = -DF_CPU=$(F_CPU) -I. 

# Cross-compilation
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
# CFLAGS += -fpack-struct
CFLAGS += -fshort-enums
CFLAGS += -ffunction-sections -fdata-sections 

# avr128da28 is not in the avr-gcc packaged for my OS 
TARGET_ARCH = -mmcu=$(MCU) \
-B $(LIBDIR)/AVR-Dx_DFP/gcc/dev/avr128da28/ \
-I $(LIBDIR)/AVR-Dx_DFP/include/
## if someday it is in mainline use
##TARGET_ARCH = -mmcu=$(MCU)

LDFLAGS = -Wl,-Map,$(TARGET).map 
LDFLAGS += -Wl,--gc-sections 

## printf() is one of the vfprintf() family of functions which does not implement floating point conversion by default
## https://www.nongnu.org/avr-libc/user-manual/group__avr__stdio.html
LDFLAGS += -Wl,-u,vfprintf -lprintf_flt -lm

.PHONY: help

# some help for the make impaired
# https://marmelab.com/blog/2016/02/29/auto-documented-makefile.html
help:
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | sort | awk 'BEGIN {FS = ":.*?## "}; {printf "\033[36m%-30s\033[0m %s\n", $$1, $$2}'

all: $(TARGET).hex $(TARGET).lst ## build the image and its related files

$(TARGET): $(TARGET).hex

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

# # This part has a built in UART based uploader (UPDI), so no bootloader is needed.
# sudo apt install python3-pip
# pip3 install pymcuprog
updi: ## upload with pymcuprog
	@echo testing for prerequesetits a false will stop make
	which python3 2>/dev/null || false
	which pymcuprog 2>/dev/null || false
	ls ../../../MacGyver/Manager/AppUpload/AppUPDImode.py 2>/dev/null || false
	ls ../../../MacGyver/Manager/AppUpload/AppUARTmode.py 2>/dev/null || false
	python3 ../../../MacGyver/Manager/AppUpload/AppUPDImode.py
	pymcuprog erase -t uart -u $(UPDI_PORT) -d $(MCU)
	pymcuprog write -t uart -u $(UPDI_PORT) -d $(MCU) --verify -f $(TARGET).hex
	python3 ../../../MacGyver/Manager/AppUpload/AppUARTmode.py

reset: ## reset AVR with pymcuprog
	@echo testing for prerequesetits a false will stop make
	which python3 2>/dev/null || false
	which pymcuprog 2>/dev/null || false
	ls ../../../MacGyver/Manager/AppUpload/AppUPDImode.py 2>/dev/null || false
	ls ../../../MacGyver/Manager/AppUpload/AppUARTmode.py 2>/dev/null || false
	python3 ../../../MacGyver/Manager/AppUpload/AppUPDImode.py
	pymcuprog reset -t uart -u $(UPDI_PORT) -d $(MCU)
	python3 ../../../MacGyver/Manager/AppUpload/AppUARTmode.py

hdrcode: ## copy header for VSCode to find run with sudo
	@echo VSCode will not look outside or cross-origin for an include,  
	@echo so I have to put the MCU header where it can find it.
	cp -u ../lib/AVR-Dx_DFP/include/avr/ioavr128da28.h /usr/lib/avr/include/avr/ioavr128da28.h
	chmod u-x /usr/lib/avr/include/avr/ioavr128da28.h

$(TARGET).elf: $(OBJECTS)
	$(CC) $(LDFLAGS) $(TARGET_ARCH) $^ -o $@
	$(SIZE) $@
	rm -f $(TARGET).o $(OBJECTS)

clean: ## remove the image and its related files
	rm -f $(TARGET).hex $(TARGET).map $(TARGET).elf $(TARGET).lst
 
%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@

builtin: ## show list of builtin (hidden) defines, some may need to be added to VScode c_cpp_properties.json
	$(CC) $(LDFLAGS) $(TARGET_ARCH) -E -dM - < /dev/null