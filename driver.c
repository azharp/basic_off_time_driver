/*
 * "Off Time Basic Driver" for ATtiny controlled flashlights
 *  Copyright (C) 2014 Alex van Heuvelen (alexvanh)
 *
 * Basic firmware demonstrating a method for using off-time to
 * switch modes on attiny13 drivers such as the nanjg drivers (a feature
 * not supported by the original firmware).
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */



/* This firmware uses off-time memory for mode switching without
 * hardware modifications on nanjg drivers. This is accomplished by
 * storing a flag in an area of memory that does not get initialized.
 * There is enough energy stored in the decoupling capacitor to
 * power the SRAM and keep data during power off for about 500ms.
 *
 * When the firmware starts a byte flag is checked. If the flashlight
 * was off for less than ~500ms all bits will still be 0. If the
 * flashlight was off longer than that some of the bits in SRAM will
 * have decayed to 1. After the flag is checked it is reset to 0.
 * Being off for less than ~500ms means the user half-pressed the
 * switch (using it as a momentary button) and intended to switch modes.
 *
 * This can be used to store any value in memory. However it is not
 * guaranteed that all of the bits will decay to 1. Checking that no
 * bits in the flag have decayed acts as a checksum and seems to be
 * enough to be reasonably certain other SRAM data is still valid.
 *
 * In order for this to work brown out detection must be enabled by
 * setting the correct fuse bits. I'm not sure why this is, maybe
 * reduced current consumption due to the reset being held once the
 * capacitor voltage drops below the threshold?
 */

#define F_CPU 4800000
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>

#define MODE_MEMORY

#ifdef MODE_MEMORY // only using eeprom if mode memory is enabled
uint8_t EEMEM MODE_P;
uint8_t EEMEM LVL_P;
#endif

// store in uninitialized memory so it will not be overwritten and
// can still be read at startup after short (<500ms) power off
// decay used to tell if user did a short press.
volatile uint8_t noinit_decay __attribute__ ((section (".noinit")));
volatile uint8_t noinit_mode __attribute__ ((section (".noinit")));
// extended modes
volatile uint8_t noinit_short __attribute__ ((section (".noinit")));
// extended mode enable, 0 if in regular mode group
volatile uint8_t noinit_strobe __attribute__ ((section (".noinit")));
// extended mode
volatile uint8_t noinit_strobe_mode __attribute__ ((section (".noinit")));

// PWM configuration
#define PWM_PIN PB1
#define PWM_LVL OCR0B
#define PWM_TCR 0x21
#define PWM_SCL 0x01

// This will be the same as the PWM_PIN on a stock driver
#define STROBE_PIN PB1

// strobe using the STROBE_PIN. Note that PWM on that pin should not be
// set up, or it should be disabled before calling this function.
static void inline strobe()
{
    while (1){
        PORTB |= _BV(STROBE_PIN); // on
        _delay_ms(20);
        PORTB &= ~_BV(STROBE_PIN); // off
        _delay_ms(90);
    }
}

// add beacon mode of 20ms on, 3000ms off
static void inline beacon()
{
    while (1){
        PORTB |= _BV(STROBE_PIN); // on
        _delay_ms(20);
        PORTB &= ~_BV(STROBE_PIN); // off
        _delay_ms(3000);
    }
}

static void inline sleep_ms(uint16_t ms)
{
    while(ms >= 1){
        _delay_ms(1);
        --ms;
    }
}

// Variable strobe
// strobe using the STROBE_PIN. Note that PWM on that pin should not be
// set up, or it should be disabled before calling this function.
static void inline strobe2(uint8_t on, uint8_t off)
{
    while (1){
        PORTB |= _BV(STROBE_PIN); // on
        sleep_ms(on);
        PORTB &= ~_BV(STROBE_PIN); // off
        sleep_ms(off);
    }
}

int main(void)
{
    if (noinit_decay) // not short press, all noinit data invalid
    {
        noinit_mode = 0;
        noinit_short = 0; // reset short counter
        noinit_strobe = 0;
        noinit_strobe_mode = 0;

        #ifdef  MODE_MEMORY // get mode from eeprom
        noinit_mode =  eeprom_read_byte(&MODE_P);
        #endif
    }
    else
    {
        ++noinit_mode;
        ++noinit_short;
    }

	noinit_decay = 0;

    // mode needs to loop back around
    // (or the mode is invalid)
    if (noinit_mode > 3) // there are 4 modes
    {
        noinit_mode = 0;
    }
    
    if (noinit_short > 2 && !noinit_strobe)
    {
        noinit_strobe = 1;
        noinit_strobe_mode = 0;
    }

    if (noinit_strobe_mode > 0) // only 1 strobe mode, could add more...
    {
        noinit_strobe_mode = 0; // loop back to first mode
    }

    //setup pins for output. Note that these pins could be the same pin
    DDRB |= _BV(PWM_PIN) | _BV(STROBE_PIN);

    // extended modes, 1 for now, leaving extra code in case I want to
    // add more strobes later
    if (noinit_strobe)
    {
        switch(noinit_strobe_mode){
            case 0:
            beacon();
            break;
        }
    }

    // Initialise PWM on output pin and set level to zero
    TCCR0A = PWM_TCR;
    TCCR0B = PWM_SCL;

    PWM_LVL = 0;

    switch(noinit_mode){
        case 0:
        PWM_LVL = 0x04;
        break;
        case 1:
        PWM_LVL = 0xE;
        break;
        case 2:
        PWM_LVL = 0x64;
        break;
        case 3:
        PWM_LVL = 0xFF;
        break;
    }

    // keep track of the number of very short on times
    // used to decide when to go into strobe mode
    _delay_ms(25); // on for too long
    noinit_short = 0; // reset short press counter
    
    #ifdef MODE_MEMORY // remember mode in eeprom
    eeprom_busy_wait(); //make sure eeprom is ready
    eeprom_write_byte(&MODE_P, noinit_mode); // save mode
    #endif
    while(1);
    return 0;
}