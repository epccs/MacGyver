# Uart0_hello is used to kick the tires
TARGET = Uart0_hello
LIBDIR = ../lib
OBJECTS = main.o 

# Chip and project-specific global definitions
MCU   =  atmega4809
#F_CPU = 16000000UL  
#BAUD  =  38400UL
#CPPFLAGS = -DF_CPU=$(F_CPU) -DBAUD=$(BAUD) -I. 

# Cross-compilation
CC = ~/Samba/avr8-3.6.2/avr8-gnu-toolchain-linux_x86_64/bin/avr-gcc
OBJCOPY = ~/Samba/avr8-3.6.2/avr8-gnu-toolchain-linux_x86_64/bin/avr-objcopy
OBJDUMP = ~/Samba/avr8-3.6.2/avr8-gnu-toolchain-linux_x86_64/bin/avr-objdump
SIZE = ~/Samba/avr8-3.6.2/avr8-gnu-toolchain-linux_x86_64/bin/avr-size

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

# atmega4809 is not in the avr-gcc packaged for my OS so I used the sideloaded toolchain from Microchip
# https://gcc.gnu.org/onlinedocs/gcc/Option-Summary.html
# option -Bprefix : This option specifies where to find the executables, libraries, include files, and data files of the compiler itself.
# option -Ldir : Add directory dir to the list of directories to be searched for -l
# option -I : These options specify directories to search for header files, for libraries and for parts of the compiler:
# newer GCC  would want the -isystem option rather than -I
TARGET_ARCH = -mmcu=$(MCU) \
-B $(LIBDIR)/Atmel.ATmega_DFP.1.4.331.atpack/gcc/dev/atmega4809/ \
-I ~/Samba/avr8-3.6.2/avr8-gnu-toolchain-linux_x86_64/avr/include/ \
-I ~/Samba/avr8-3.6.2/avr8-gnu-toolchain-linux_x86_64/lib/gcc/avr/5.4.0/include/ \
-I $(LIBDIR)/Atmel.ATmega_DFP.1.4.331.atpack/include/
## if someday it is in mainline use
##TARGET_ARCH = -mmcu=$(MCU)


LDFLAGS = -Wl,-Map,$(TARGET).map 
LDFLAGS += -Wl,--gc-sections 

all: $(TARGET).hex $(TARGET).lst

$(TARGET): $(TARGET).hex

$(TARGET).hex: $(TARGET).elf
	$(OBJCOPY) -j .text -j .data -O ihex $< $@

# # This part has a built in uploader as part of UPDI, so no bootloader is needed but avrdude is not set up to upload to this interface so...
# sudo apt install python3-pip
# pip3 install serial
# # place pyupdi next to your project repository
# git clone https://github.com/mraardvark/pyupdi
# # fuses, what, what?
updi: $(TARGET).hex ## use the built in UPDI programing interface 
	shell sh -c 'which python3 2>/dev/null || echo install python3'
	shell sh -c 'ls ../../pyupdi/pyupdi.py 2>/dev/null || echo clone pyupdi'
	python3 ../../pyupdi/pyupdi.py -v -d $(MCU) -c $(UPDI_PORT) -b 115200 -e -f $(TARGET).hex

$(TARGET).elf: $(OBJECTS)
	$(CC) $(LDFLAGS) $(TARGET_ARCH) $^ -o $@
	$(SIZE) -C --mcu=$(MCU) $@
	rm -f $(TARGET).o $(OBJECTS)

clean: 
	rm -f $(TARGET).hex $(TARGET).map $(TARGET).elf $(TARGET).lst
 
%.lst: %.elf
	$(OBJDUMP) -h -S $< > $@
