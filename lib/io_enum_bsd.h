/* IO Library using enum
Copyright (C) 2020 Ronald Sutherland
 
Permission to use, copy, modify, and/or distribute this software for any purpose with or without fee is hereby granted.

THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES 
WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF 
MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE 
FOR ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY 
DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, 
WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, 
ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

https://en.wikipedia.org/wiki/BSD_licenses#0-clause_license_(%22Zero_Clause_BSD%22)

Editors like VScode with IntelliSense can show appropriate options.
The compiler will reject code that is an incorrect type.
*/
#ifndef IO_Enum_h
#define IO_Enum_h

#if defined(__AVR_ATmega4809__)

// Direction is used to program the IO as an input or output
typedef enum direction {
    INPUT,
    OUTPUT
} DIRECTION_t;

/* Configuration:  PORT.PIN0CTRL   has three bits for Input Sense Configuration ISC[2:0]
    PORT.PIN0CTRL  bit masks and bit positions
    enum can only promote to an integer but these (io_config_isc+io_config_pullup+io_config_invert)  
    need to be cast into type register8_t.

// this is in iom4809.h
typedef enum PORT_ISC_enum
{
    PORT_ISC_INTDISABLE_gc = (0x00<<0),  // Interrupt disabled but input buffer enabled 
    PORT_ISC_BOTHEDGES_gc = (0x01<<0),  // Sense Both Edges 
    PORT_ISC_RISING_gc = (0x02<<0),  // Sense Rising Edge
    PORT_ISC_FALLING_gc = (0x03<<0),  // Sense Falling Edge
    PORT_ISC_INPUT_DISABLE_gc = (0x04<<0),  // Digital Input Buffer disabled
    PORT_ISC_LEVEL_gc = (0x05<<0),  // Sense low Level
} PORT_ISC_t;
*/

// Configuration:  PORT.PIN0CTRL   has one bit (3) for Pullup Enable
typedef enum PORT_PULLUP_enum {
    // Pullup disabled
    PORT_PULLUP_DISABLE = 0,
    // Pullup enabled
    PORT_PULLUP_ENABLE = 0x08
} PORT_PULLUP_t;

/* Configuration:  PORT.PIN0CTRL   has one bit (7) for Inverted I/O
*/
typedef enum PORT_INVERT_enum {
    // Input and output values are not inverted
    PORT_INVERT_NORMAL = 0,
    // Input and output values are inverted
    PORT_INVERT_INVERTED = 0x80
} PORT_INVERT_t;

// Logic Level
typedef enum logic_level {
    LOW,
    HIGH
} LOGIC_LEVEL_t;

// enumeraiton names for MCU IO is on schematic
typedef enum mcu_io {
    AIN0,
    AIN1,
    AIN2,
    AIN3,
    AIN4,
    AIN5,
    AIN6,
    AIN7,
    AIN8,
    AIN9,
    AIN10,
    AIN11,
    SDA1, // TWI slave only
    SCL1, // TWI slave only
    AIN14,
    AIN15,
    TX0,
    RX0,
    SDA0,  // TWI master or slave
    SCL0,  // TWI master or slave
    MOSI,
    MISO,
    SCK,
    nSS,
    TX1,
    RX1,
    PWR_RPI_CNTL, // pull LOW to set latch, floating latch will hold, pull HIGH to clear latch and stop power
    SHUTDOWN_CNTL, // set with weak pull up to allow manual swithch to control, pull down to hault R-Pi 
    PC4,
    PC5,
    TX2,
    RX2,
    nRESET,  // GPIO if SYSCFG0 bit 3 RSTPINCFG reads as 1 thus  fuse is unprogrammed and pin is in GPIO mode.
                  // https://www.avrfreaks.net/forum/atmega4809-reset-pin-fuse-bug
    END_OF_IO
} IO_t;

/* The port and pin mask are needed.  iom4809.h has a PORT_struct that the io functions use to map the register functions.

typedef struct PORT_struct
{
    register8_t DIR;  // Data Direction 
    register8_t DIRSET;  // Data Direction Set (output)
    register8_t DIRCLR;  // Data Direction Clear (input)
    register8_t DIRTGL;  // Data Direction Toggle 
    register8_t OUT;  // Output Value 
    register8_t OUTSET;  // Output Value Set 
    register8_t OUTCLR;  // Output Value Clear 
    register8_t OUTTGL;  // Output Value Toggle 
    register8_t IN;  // Input Value 
    register8_t INTFLAGS;  // Interrupt Flags 
    register8_t PORTCTRL;  // Port Control 
    register8_t reserved_1[5];
    register8_t PIN0CTRL;  // Pin 0 Control 
    register8_t PIN1CTRL;  // Pin 1 Control 
    register8_t PIN2CTRL;  // Pin 2 Control 
    register8_t PIN3CTRL;  // Pin 3 Control 
    register8_t PIN4CTRL;  // Pin 4 Control 
    register8_t PIN5CTRL;  // Pin 5 Control 
    register8_t PIN6CTRL;  // Pin 6 Control 
    register8_t PIN7CTRL;  // Pin 7 Control 
    register8_t reserved_2[8];
} PORT_t;
*/
struct IO_Map { // https://yarchive.net/comp/linux/typedefs.html
    PORT_t *port; // pointer to above struct
    uint8_t mask; // Generic Port Pin mask 
};

const static struct IO_Map ioMap[END_OF_IO] = {
    [AIN0] = { .port= &PORTD, .mask= PIN0_bm },
    [AIN1] = { .port= &PORTD, .mask= PIN1_bm },
    [AIN2] = { .port= &PORTD, .mask= PIN2_bm },
    [AIN3] = { .port= &PORTD, .mask= PIN3_bm },
    [AIN4] = { .port= &PORTD, .mask= PIN4_bm },
    [AIN5] = { .port= &PORTD, .mask= PIN5_bm },
    [AIN6] = { .port= &PORTD, .mask= PIN6_bm },
    [AIN7] = { .port= &PORTD, .mask= PIN7_bm },
    [AIN8] = { .port= &PORTE, .mask= PIN0_bm },
    [AIN9] = { .port= &PORTE, .mask= PIN1_bm },
    [AIN10] = { .port= &PORTE, .mask= PIN2_bm },
    [AIN11] = { .port= &PORTE, .mask= PIN3_bm },
    [SDA1] = { .port= &PORTF, .mask= PIN2_bm },
    [SCL1] = { .port= &PORTF, .mask= PIN3_bm },
    [AIN14] = { .port= &PORTF, .mask= PIN4_bm },
    [AIN15] = { .port= &PORTF, .mask= PIN5_bm },
    [TX0] = { .port= &PORTA, .mask= PIN0_bm },
    [RX0] = { .port= &PORTA, .mask= PIN1_bm },
    [SDA0] = { .port= &PORTA, .mask= PIN2_bm },
    [SCL0] = { .port= &PORTA, .mask= PIN3_bm },
    [MOSI] = { .port= &PORTA, .mask= PIN4_bm },
    [MISO] = { .port= &PORTA, .mask= PIN5_bm },
    [SCK] = { .port= &PORTA, .mask= PIN6_bm },
    [nSS] = { .port= &PORTA, .mask= PIN7_bm },
    [TX1] = { .port= &PORTC, .mask= PIN0_bm },
    [RX1] = { .port= &PORTC, .mask= PIN1_bm },
    [PWR_RPI_CNTL] = { .port= &PORTC, .mask= PIN2_bm }, 
    [SHUTDOWN_CNTL] = { .port= &PORTC, .mask= PIN3_bm },
    [PC4] = { .port= &PORTC, .mask= PIN4_bm },
    [PC5] = { .port= &PORTC, .mask= PIN5_bm },
    [TX2] = { .port= &PORTF, .mask= PIN0_bm },
    [RX2] = { .port= &PORTF, .mask= PIN1_bm },
    [nRESET] = { .port= &PORTF, .mask= PIN6_bm }
};

#else
#   error this is for an mega4809 on a PCB board, see https://github.com/epccs/PiUpdi
#endif


static inline __attribute__((always_inline))
volatile PORT_t* portReg(IO_t io) {
  return ioMap[io].port;
}

static inline __attribute__((always_inline))
uint8_t ioMask(IO_t io) {
  return ioMap[io].mask;
}

// read value from IO input bit and return its bool value
static inline __attribute__((always_inline))
bool ioRead(IO_t io) 
{
    return (portReg(io)->IN & ioMask(io));
}

// set or clear IO output
static inline __attribute__((always_inline))
void ioWrite(IO_t io, LOGIC_LEVEL_t level) {
    if (level == HIGH) {
        portReg(io)->OUTSET = ioMask(io);
    }
    else {
        portReg(io)->OUTCLR = ioMask(io);
    }
}

// toggle io 
//  Toggling an INVERTED pin causes an edge, which can be detected by 
//  all peripherals using the pin, and by interrupts or Events if enabled.
static inline __attribute__((always_inline))
void ioToggle(IO_t io) {
    portReg(io)->OUTTGL = ioMask(io);
}

// set io direction (INPUT or OUTPUT).
static inline __attribute__((always_inline))
void ioDir(IO_t io, DIRECTION_t dir) {
    if (dir == OUTPUT) {
        portReg(io)->DIRSET = ioMask(io); // write bit n of the PORTx.DIR register to '1'
    }
    else {
        portReg(io)->DIRCLR = ioMask(io); // write bit n of the PORTx.DIR register to '0'
    }
}

// io control settings
static inline __attribute__((always_inline))
void ioCntl(IO_t io, PORT_ISC_t isc, PORT_PULLUP_t pu, PORT_INVERT_t inv) {
    // is there a better way? should PINnCTRL be in ioMap array (it would use more memory I think)
    if (ioMask(io) == PIN0_bm) portReg(io)->PIN0CTRL = (register8_t) (isc+pu+inv);
    if (ioMask(io) == PIN1_bm) portReg(io)->PIN1CTRL = (register8_t) (isc+pu+inv);
    if (ioMask(io) == PIN2_bm) portReg(io)->PIN2CTRL = (register8_t) (isc+pu+inv);
    if (ioMask(io) == PIN3_bm) portReg(io)->PIN3CTRL = (register8_t) (isc+pu+inv);
    if (ioMask(io) == PIN4_bm) portReg(io)->PIN4CTRL = (register8_t) (isc+pu+inv);
    if (ioMask(io) == PIN5_bm) portReg(io)->PIN5CTRL = (register8_t) (isc+pu+inv);
    if (ioMask(io) == PIN6_bm) portReg(io)->PIN6CTRL = (register8_t) (isc+pu+inv);
    if (ioMask(io) == PIN7_bm) portReg(io)->PIN7CTRL = (register8_t) (isc+pu+inv);
}


#endif  // IO_Enum_h

