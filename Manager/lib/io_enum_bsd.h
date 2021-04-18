/* IO Library using enum
Copyright (C) 2021 Ronald Sutherland
 
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
Some compilers can reject code that is an incorrect type.

Some referance material to help get the needed SBI/CBI instructions
https://www.avrfreaks.net/forum/c-gpio-class-inheritance-problems
https://github.com/greiman/SdFat-beta/blob/master/src/DigitalIO/DigitalPin.h
*/
#ifndef IO_Enum_h
#define IO_Enum_h

// Direction is used to program the IO as an input or output
typedef enum DIRECTION_enum 
{
    DIRECTION_INPUT,
    DIRECTION_OUTPUT
} DIRECTION_t;

/* Configuration:  PORT.PIN0CTRL   has three bits for Input Sense Configuration ISC[2:0]
    PORT.PIN0CTRL  bit masks and bit positions
    enum can only promote to an integer but these (io_config_isc+io_config_pullup+io_config_invert)  
    need to be cast into type register8_t.

// Input/Sense Configuration select is in ioavr128da28.h
typedef enum PORT_ISC_enum
{
    PORT_ISC_INTDISABLE_gc = (0x00<<0),  // Interrupt disabled but input buffer enabled 
    PORT_ISC_BOTHEDGES_gc = (0x01<<0),  // Sense Both Edges 
    PORT_ISC_RISING_gc = (0x02<<0),  // Sense Rising Edge
    PORT_ISC_FALLING_gc = (0x03<<0),  // Sense Falling Edge
    PORT_ISC_INPUT_DISABLE_gc = (0x04<<0),  // GPIO Input Buffer disabled
    PORT_ISC_LEVEL_gc = (0x05<<0),  // Sense low Level
} PORT_ISC_t;
*/

// Configuration:  PORT.PIN0CTRL   has one bit (3) for Pullup Enable
typedef enum PORT_PULLUP_enum 
{
    // Pullup disabled
    PORT_PULLUP_DISABLE = 0,
    // Pullup enabled
    PORT_PULLUP_ENABLE = PORT_PULLUPEN_bm 
} PORT_PULLUP_t;

/* Configuration:  PORT.PIN0CTRL   has one bit (7) for Inverted I/O
*/
typedef enum PORT_INVERT_enum 
{
    // Input and output values are not inverted
    PORT_INVERT_NORMAL = 0,
    // Input and output values are inverted
    PORT_INVERT_INVERTED = PORT_INVEN_bm
} PORT_INVERT_t;

// Logic Level
typedef enum LOGIC_LEVEL_enum 
{
    LOGIC_LEVEL_LOW,
    LOGIC_LEVEL_HIGH
} LOGIC_LEVEL_t;

// enumeraiton names for the MCU_IO_<node> is on schematic
typedef enum MCU_IO_enum 
{
    MCU_IO_ALT_I, // Analog channel 1, PD1 has analog channel to measure alternate current with 0.01 Ohm sense and a gain of 50 amplifier
    MCU_IO_ALT_V, // Analog channel 2, PD2 has analog channel to measued alternate voltage with 100k and 10k Ohm divider
    MCU_IO_PWR_I, // Analog channel 3, PD3 has analog channel to measure power current with 0.068 Ohm sense and a gain of 50 amplifier
    MCU_IO_PWR_V, // Analog channel 4, PD4 has analog channel to measue power voltage with 100k and 15.8k Ohm divider
    MCU_IO_RX_nRE, // PD5 controls RX pair transceiver receiver disable.
    MCU_IO_RX_DE, // PD6 controls RX pair transceiver driver enable
    MCU_IO_MGR_LED, // PD7 has red LED for manager status
    //PA0(XTALHF1/TX0) pin is not mapped
    //PA1(XTALHF2/RX0) pin is not mapped
    MCU_IO_PIPWR_EN, // PA2 has a 10k Ohm pull-up to enable power to the R-Pi (SBC), pull down with push-pull (OUTPUT mode) to turn off power. 
    MCU_IO_ALT_EN, // PA3 can pull up to enable the alternat power input, it has a 10k pull down on the board.
    MCU_IO_TX_nRE, // PA4 controls TX pair transceiver receiver disable.
    MCU_IO_TX_DE, // PA5 controls TX pair transceiver driver enable.
    MCU_IO_OOB_nRE, // PA6 controls OOB pair transceiver receiver disable.
    MCU_IO_OOB_DE, // PA7 controls OOB pair transceiver driver enable.
    MCU_IO_TX1, // PC0 is TX1 which has MVIO and is a direct crossover to R-Pi serial RX
    MCU_IO_RX1, // PC1 is RX1 which has MVIO and is a direct crossover to R-Pi serial TX
    MCU_IO_SMBUS_SDA, // PC2 has MVIO and is an SMBus SDA slave that connects the manager to the R-Pi (SBC).
    MCU_IO_SMBUS_SCL, // PC3 has MVIO and is an SMBus SCL slave that connects the manager to the R-Pi (SBC).
    MCU_IO_TO_RPU_OOB,  // PF0(TX2) used with half-duplex OOB pair for multi-drop bus state.
    MCU_IO_FROM_RPU_OOB,  // PF1(RX2) used with half duplex OOB pair
    MCU_IO_I2C_SDA, // PF2 is an I2C SDA slave and connects the manager with application uC.
    MCU_IO_I2C_SCL, // PF3 is an I2C SCL slave and connects the manager with application uC.
    MCU_IO_APP_UART, // PF4 send RPU RX and TX to application UART.
    MCU_IO_APP_UPDI, // PF5 send RPU RX and TX to application UPDI programing pin.
    MCU_IO_SHUTDOWN, // PF6 needs a weak pullup enabled so a manual switch can HALT the R-Pi on BCM6 (pin 31 on SBC header) 
    MCU_IO_END
} MCU_IO_t;

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
struct IO_Map { 
    // https://yarchive.net/comp/linux/typedefs.html
    PORT_t *port; // pointer to above struct
    uint8_t mask; // Generic Port Pin mask 
};

const static struct IO_Map ioMap[MCU_IO_END] = {
    [MCU_IO_ALT_I] = { .port= &PORTD, .mask= PIN1_bm },
    [MCU_IO_ALT_V] = { .port= &PORTD, .mask= PIN2_bm },
    [MCU_IO_PWR_I] = { .port= &PORTD, .mask= PIN3_bm },
    [MCU_IO_PWR_V] = { .port= &PORTD, .mask= PIN4_bm },
    [MCU_IO_RX_nRE] = { .port= &PORTD, .mask= PIN5_bm },
    [MCU_IO_RX_DE] = { .port= &PORTD, .mask= PIN6_bm },
    [MCU_IO_MGR_LED] = { .port= &PORTD, .mask= PIN7_bm },
    [MCU_IO_PIPWR_EN] = { .port= &PORTA, .mask= PIN2_bm },
    [MCU_IO_ALT_EN] = { .port= &PORTA, .mask= PIN3_bm },
    [MCU_IO_TX_nRE] = { .port= &PORTA, .mask= PIN4_bm },
    [MCU_IO_TX_DE] = { .port= &PORTA, .mask= PIN5_bm },
    [MCU_IO_OOB_nRE] = { .port= &PORTA, .mask= PIN6_bm },
    [MCU_IO_OOB_DE] = { .port= &PORTA, .mask= PIN7_bm },
    [MCU_IO_TX1] = { .port= &PORTC, .mask= PIN0_bm },
    [MCU_IO_RX1] = { .port= &PORTC, .mask= PIN1_bm },
    [MCU_IO_SMBUS_SDA] = { .port= &PORTC, .mask= PIN2_bm }, 
    [MCU_IO_SMBUS_SCL] = { .port= &PORTC, .mask= PIN3_bm },
    [MCU_IO_TO_RPU_OOB] = { .port= &PORTF, .mask= PIN0_bm },
    [MCU_IO_FROM_RPU_OOB] = { .port= &PORTF, .mask= PIN1_bm },
    [MCU_IO_I2C_SDA] = { .port= &PORTF, .mask= PIN2_bm },
    [MCU_IO_I2C_SCL] = { .port= &PORTF, .mask= PIN3_bm },
    [MCU_IO_APP_UART] = { .port= &PORTF, .mask= PIN4_bm },
    [MCU_IO_APP_UPDI] = { .port= &PORTF, .mask= PIN5_bm },
    [MCU_IO_SHUTDOWN] = { .port= &PORTF, .mask= PIN6_bm }
};

static inline __attribute__((always_inline))
volatile PORT_t* portReg(MCU_IO_t io) 
{
  return ioMap[io].port;
}

static inline __attribute__((always_inline))
uint8_t ioMask(MCU_IO_t io) 
{
  return ioMap[io].mask;
}

// read value from IO input bit and return its bool value
static inline __attribute__((always_inline))
bool ioRead(MCU_IO_t io) 
{
    return (portReg(io)->IN & ioMask(io));
}

// set or clear IO output
static inline __attribute__((always_inline))
void ioWrite(MCU_IO_t io, LOGIC_LEVEL_t level) 
{
    if (level == LOGIC_LEVEL_HIGH) 
    {
        portReg(io)->OUTSET = ioMask(io);
    }
    else 
    {
        portReg(io)->OUTCLR = ioMask(io);
    }
}

// toggle io 
//  Toggling an INVERTED pin causes an edge, which can be detected by 
//  all peripherals using the pin, and by interrupts or Events if enabled.
static inline __attribute__((always_inline))
void ioToggle(MCU_IO_t io) 
{
    portReg(io)->OUTTGL = ioMask(io);
}

// set io direction (DIRECTION_INPUT or DIRECTION_OUTPUT).
static inline __attribute__((always_inline))
void ioDir(MCU_IO_t io, DIRECTION_t dir) 
{
    if (dir == DIRECTION_OUTPUT) 
    {
        portReg(io)->DIRSET = ioMask(io); // write bit n of the PORTx.DIR register to '1'
    }
    else 
    {
        portReg(io)->DIRCLR = ioMask(io); // write bit n of the PORTx.DIR register to '0'
    }
}

/*  io control settings

    isc is for PINnCTRL Input/Sense Configuration select bits
    PORT_ISC_INTDISABLE_gc is for Interrupt disabled but input buffer enabled
    PORT_ISC_BOTHEDGES_gc is for Sense Both Edges
    PORT_ISC_RISING_gc is for Sense Rising Edge 
    PORT_ISC_FALLING_gc is for Sense Falling Edge 
    PORT_ISC_INPUT_DISABLE_gc is for Digital Input Buffer disabled
    PORT_ISC_LEVEL_gc is for Sense low Level

    pu is for PINnCTRL bit (3) for Pullup Enable
    PORT_PULLUP_DISABLE
    PORT_PULLUP_ENABLE

    inv is for PINnCTRL bit (7) for Inverted I/O
    PORT_INVERT_NORMAL
    PORT_INVERT_INVERTED
*/ 
static inline __attribute__((always_inline))
void ioCntl(MCU_IO_t io, PORT_ISC_t isc, PORT_PULLUP_t pu, PORT_INVERT_t inv) 
{
    switch (ioMask(io))
    {
    case PIN0_bm:
        portReg(io)->PIN0CTRL = (register8_t) (isc+pu+inv);
        break;
    case PIN1_bm:
        portReg(io)->PIN1CTRL = (register8_t) (isc+pu+inv);
        break;
    case PIN2_bm:
        portReg(io)->PIN2CTRL = (register8_t) (isc+pu+inv);
        break;
    case PIN3_bm:
        portReg(io)->PIN3CTRL = (register8_t) (isc+pu+inv);
        break;
    case PIN4_bm:
        portReg(io)->PIN4CTRL = (register8_t) (isc+pu+inv);
        break;
    case PIN5_bm:
        portReg(io)->PIN5CTRL = (register8_t) (isc+pu+inv);
        break;
    case PIN6_bm:
        portReg(io)->PIN6CTRL = (register8_t) (isc+pu+inv);
        break;
    case PIN7_bm:
        portReg(io)->PIN7CTRL = (register8_t) (isc+pu+inv);
        break;

    default:
        break;
    }
}


#endif  // IO_Enum_h

