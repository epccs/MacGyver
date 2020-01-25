/* DigitalIO Library
  Copyright (C) 2020 Ronald Sutherland
 
  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 
   Hacked from William Greiman to work in C with my board
   Functions are inspired by Wiring from Hernando Barragan
 */
#ifndef PinNum_h
#define PinNum_h

// Direction
#define INPUT 0
#define OUTPUT 1

// Configuration
// use PORT_PULLUPEN_bm, PORT_INVEN_bm, and PORT_ISC_enum from iom4809.h

// Level
#define LOW 0
#define HIGH 1

/* The port and pin mask are needed.  iom4809.h has a PORT_struct that will be used to map the register based functions.

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
struct Pin_Map { // https://yarchive.net/comp/linux/typedefs.html
    PORT_t *port; // pointer to above struct
    uint8_t mask; // Generic Port Pin mask
};

#if defined(__AVR_ATmega4809__)

#define NUM_DIGITAL_PINS 33

/* Each of the AVR Digital I/O ports is associated with registers that have functions. 
The megaAVR 0-series has more going on than the Mega series so look at
https://github.com/MicrochipTech/Getting_Started_with_GPIO
https://www.microchip.com/wwwproducts/en/ATMEGA4809

Wiring uses pin numbers to control their functions. */
const static struct Pin_Map pinMap[NUM_DIGITAL_PINS] = {
    [0] = { .port= &PORTD, .mask= PIN0_bm }, // AIN0
    [1] = { .port= &PORTD, .mask= PIN1_bm }, // AIN1
    [2] = { .port= &PORTD, .mask= PIN2_bm }, // AIN2
    [3] = { .port= &PORTD, .mask= PIN3_bm }, // AIN3
    [4] = { .port= &PORTD, .mask= PIN4_bm }, // AIN4
    [5] = { .port= &PORTD, .mask= PIN5_bm }, // AIN5
    [6] = { .port= &PORTD, .mask= PIN6_bm }, // AIN6
    [7] = { .port= &PORTD, .mask= PIN7_bm }, // AIN7
    [8] = { .port= &PORTE, .mask= PIN0_bm }, // AIN8
    [9] = { .port= &PORTE, .mask= PIN1_bm }, // AIN9
    [10] = { .port= &PORTE, .mask= PIN2_bm }, // AIN10
    [11] = { .port= &PORTE, .mask= PIN3_bm }, // AIN11
    [12] = { .port= &PORTF, .mask= PIN2_bm }, // SDA_S/AIN12
    [13] = { .port= &PORTF, .mask= PIN3_bm }, // SCL_S/AIN13
    [14] = { .port= &PORTF, .mask= PIN4_bm }, // AIN14
    [15] = { .port= &PORTF, .mask= PIN5_bm }, // AIN15
    [16] = { .port= &PORTA, .mask= PIN0_bm }, // TX0
    [17] = { .port= &PORTA, .mask= PIN1_bm }, // RX0
    [18] = { .port= &PORTA, .mask= PIN2_bm }, // SDA_M
    [19] = { .port= &PORTA, .mask= PIN3_bm }, // SCL_M
    [20] = { .port= &PORTA, .mask= PIN4_bm }, // MOSI
    [21] = { .port= &PORTA, .mask= PIN5_bm }, // MISO
    [22] = { .port= &PORTA, .mask= PIN6_bm }, // SCK
    [23] = { .port= &PORTA, .mask= PIN7_bm }, // !SS
    [24] = { .port= &PORTC, .mask= PIN0_bm }, // TX1
    [25] = { .port= &PORTC, .mask= PIN1_bm }, // RX1
    [26] = { .port= &PORTC, .mask= PIN2_bm }, // 
    [27] = { .port= &PORTC, .mask= PIN3_bm }, // 
    [28] = { .port= &PORTC, .mask= PIN4_bm }, // 
    [29] = { .port= &PORTC, .mask= PIN5_bm }, // 
    [30] = { .port= &PORTF, .mask= PIN0_bm }, // TX2
    [31] = { .port= &PORTF, .mask= PIN1_bm }, // RX2
    [32] = { .port= &PORTF, .mask= PIN6_bm } // RESET
};

#else
#   error this is for a mega4809
#endif

void badPinNumber(void)
  __attribute__((error("Bad pin number or it is not a constant")));

// dead code elimination should remove the badPinNumber function call if the pin is valid
// https://gcc.gnu.org/onlinedocs/gcc/Other-Builtins.html
static inline __attribute__((always_inline))
void badPinCheck(uint8_t pin_num) {
  if (!__builtin_constant_p(pin_num) || pin_num >= NUM_DIGITAL_PINS) {
     badPinNumber();
  }
}

static inline __attribute__((always_inline))
volatile PORT_t* portReg(uint8_t pin_num) {
  badPinCheck(pin_num);
  return pinMap[pin_num].port;
}

static inline __attribute__((always_inline))
uint8_t pinMask(uint8_t pin_num) {
  badPinCheck(pin_num);
  return pinMap[pin_num].mask;
}

/* read value from pin number return 0 or 1 */
static inline __attribute__((always_inline))
bool digitalRead(uint8_t pin_num) 
{
    return (portReg(pin_num)->IN & pinMask(pin_num));
}

/* set pin value HIGH or LOW */
static inline __attribute__((always_inline))
void digitalWrite(uint8_t pin_num, bool value_for_bit) {
    if (value_for_bit) {
        portReg(pin_num)->OUTSET = pinMask(pin_num);
    }
    else {
        portReg(pin_num)->OUTCLR = pinMask(pin_num);
    }
}

/* toggle pin number 
    Toggling an INVERTED pin causes an edge, which can be detected by 
    all peripherals using the pin, and by interrupts or Events if enabled. */
static inline __attribute__((always_inline))
void digitalToggle(uint8_t pin_num) {
    portReg(pin_num)->OUTTGL = pinMask(pin_num);
}

/* set pin mode INPUT or OUTPUT. */
static inline __attribute__((always_inline))
void pinMode(uint8_t pin_num, bool output_mode) {
    if (output_mode) {
        portReg(pin_num)->DIRSET = pinMask(pin_num);
    }
    else {
        portReg(pin_num)->DIRCLR = pinMask(pin_num);
    }
}

/* pin control settings from iom4809.h
#define PORT_ISC0_bm  (1<<0)  // Input/Sense Configuration bit 0 mask. 
#define PORT_ISC1_bm  (1<<1)  // Input/Sense Configuration bit 1 mask.
#define PORT_ISC2_bm  (1<<2)  // Input/Sense Configuration bit 2 mask.
#define PORT_PULLUPEN_bm  0x08  // Pullup enable bit mask.
#define PORT_INVEN_bm  0x80  // Inverted I/O Enable bit mask.

// ISC 0..5
// Input/Sense Configuration select is in iom4809.h
typedef enum PORT_ISC_enum
{
    PORT_ISC_INTDISABLE_gc = (0x00<<0),  // Interrupt disabled but input buffer enabled 
    PORT_ISC_BOTHEDGES_gc = (0x01<<0),  // Sense Both Edges 
    PORT_ISC_RISING_gc = (0x02<<0),  // Sense Rising Edge
    PORT_ISC_FALLING_gc = (0x03<<0),  // Sense Falling Edge
    PORT_ISC_INPUT_DISABLE_gc = (0x04<<0),  // Digital Input Buffer disabled
    PORT_ISC_LEVEL_gc = (0x05<<0),  // Sense low Level
} PORT_ISC_t;

How to use: set pin 1 as pullup
pinControl(1, PORT_PULLUPEN_bm)

How to use: also set pin 1 as pullup with ISC set for both edges
pinControl(1, PORT_PULLUPEN_bm & PORT_ISC_BOTHEDGES_gc)
*/
static inline __attribute__((always_inline))
void pinControl(uint8_t pin_num, register8_t cntl_mode) {
    // is there a better way?
    if (pinMask(pin_num) == PIN0_bm) portReg(pin_num)->PIN0CTRL = cntl_mode;
    if (pinMask(pin_num) == PIN1_bm) portReg(pin_num)->PIN1CTRL = cntl_mode;
    if (pinMask(pin_num) == PIN2_bm) portReg(pin_num)->PIN2CTRL = cntl_mode;
    if (pinMask(pin_num) == PIN3_bm) portReg(pin_num)->PIN3CTRL = cntl_mode;
    if (pinMask(pin_num) == PIN4_bm) portReg(pin_num)->PIN4CTRL = cntl_mode;
    if (pinMask(pin_num) == PIN5_bm) portReg(pin_num)->PIN5CTRL = cntl_mode;
    if (pinMask(pin_num) == PIN6_bm) portReg(pin_num)->PIN6CTRL = cntl_mode;
    if (pinMask(pin_num) == PIN7_bm) portReg(pin_num)->PIN7CTRL = cntl_mode;
}
#endif  // PinNum_h

