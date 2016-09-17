/*
	mk2_FW_mod 20150608 - based on source code listed below

	FEATURES

	- off-time functions without hardware modification
	- low-voltage monitor; 4 sec low light warning and shutdown at 2.9V
	- battery check mode
	- 7 modes + hidden strobe: moon, low, med, max, batt, beacon (10/1000ms), ramp
	- memory is disabled because it is less relevant in off-time operation

	NOTES

	Battery check mode blinks:
	- 0 blinks: < 3.0V
	- 1 blink: 3.0 - 3.3V
	- 2 blinks: 3.3 - 3.6V
	- 3 blinks: 3.6 - 3.9V
	- 4 blinks: 3.9 - 4.2V
	- 5 blinks: > 4.2V

	AK47A test
	0x03	2mA
	0x05	7mA
	0x06	9mA
	0x08	13mA
	0x10	50mA
	0x40	240mA
	0xFF	1A
	strobe2(10, 1000); // ~10mA

    _delay_ms(25); // on for too long ; noinit_short = 0; // reset short press counter
	was moved before the switch to improve mode detection

	flash: avrdude -p t13 -c usbasp -u -U flash:w:code.hex:a -U lfuse:w:0x79:m -U hfuse:w:0xed:m
*/


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



/*
 * Original author: JonnyC
 * Modifications: ToyKeeper / Selene Scriven
 *
 * NANJG 105C Diagram
 *           ---
 *         -|   |- VCC
 *  Star 4 -|   |- Voltage ADC
 *  Star 3 -|   |- PWM
 *     GND -|   |- Star 2
 *           ---
 *
 * CPU speed is 4.8Mhz without the 8x divider when low fuse is 0x79
 *
 * define F_CPU 4800000  CPU: 4.8MHz  PWM: 9.4kHz       ####### use low fuse: 0x79  #######
 * 
 * Above PWM speeds are for phase-correct PWM.  This program uses Fast-PWM,
 * which when the CPU is 4.8MHz will be 18.75 kHz
 *
 * VOLTAGE
 *      Resistor values for voltage divider (reference BLF-VLD README for more info)
 *      Reference voltage can be anywhere from 1.0 to 1.2, so this cannot be all that accurate
 *
 *           VCC
 *            |
 *           Vd (~.25 v drop from protection diode)
 *            |
 *          1912 (R1 19,100 ohms)
 *            |
 *            |---- PB2 from MCU
 *            |
 *          4701 (R2 4,700 ohms)
 *            |
 *           GND
 *
 *      ADC = ((V_bat - V_diode) * R2   * 255) / ((R1    + R2  ) * V_ref)
 *      125 = ((3.0   - .25    ) * 4700 * 255) / ((19100 + 4700) * 1.1  )
 *      121 = ((2.9   - .25    ) * 4700 * 255) / ((19100 + 4700) * 1.1  )
 *
 *      Well 125 and 121 were too close, so it shut off right after lowering to low mode, so I went with
 *      130 and 120
 *
 *      To find out what value to use, plug in the target voltage (V) to this equation
 *          value = (V * 4700 * 255) / (23800 * 1.1)
 *
 */



#define F_CPU 4800000
#include <avr/io.h>
#include <stdlib.h>
#include <util/delay.h>
#include <avr/eeprom.h>
#include <avr/pgmspace.h>
#include <avr/sleep.h>

//#define MODE_MEMORY

#ifdef MODE_MEMORY // only using eeprom if mode memory is enabled
uint8_t EEMEM MODE_P;
uint8_t EEMEM LVL_P;
#endif

// store in uninitialized memory so it will not be overwritten and
// can still be read at startup after short (<500ms) power off
// decay used to tell if user did a short press.
volatile uint8_t noinit_decay __attribute__ ((section (".noinit")));
volatile uint8_t noinit_mode __attribute__ ((section (".noinit")));
// pwm level selected by ramping function
volatile uint8_t noinit_lvl __attribute__ ((section (".noinit")));
// number of times light was on for a short period, used to enter
// extended modes
volatile uint8_t noinit_short __attribute__ ((section (".noinit")));
// extended mode enable, 0 if in regular mode group
volatile uint8_t noinit_strobe __attribute__ ((section (".noinit")));
// extended mode
volatile uint8_t noinit_strobe_mode __attribute__ ((section (".noinit")));

// PWM configuration
// Set timer to do PWM for correct output pin and set prescaler timing
//TCCR0A = 0x23; // phase corrected PWM is 0x21 for PB1, fast-PWM is 0x23
//TCCR0B = 0x01; // pre-scaler for timer (1 => 1, 2 => 8, 3 => 64...)
#define PWM_PIN PB1
#define PWM_LVL OCR0B
#define PWM_TCR 0x21
#define PWM_SCL 0x01

// This will be the same as the PWM_PIN on a stock driver
#define STROBE_PIN PB1

/* Ramping configuration.
 * Configure the LUT used for the ramping function and the delay between
 * steps of the ramp.
 */

// delay in ms between each ramp step
#define RAMP_DELAY 30 //30

#define SINUSOID 4, 4, 5, 6, 8, 10, 13, 16, 20, 24, 28, 33, 39, 44, 50, 57, 63, 70, 77, 85, 92, 100, 108, 116, 124, 131, 139, 147, 155, 163, 171, 178, 185, 192, 199, 206, 212, 218, 223, 228, 233, 237, 241, 244, 247, 250, 252, 253, 254, 255
// natural log of a sinusoid
#define LN_SINUSOID 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 6, 7, 7, 8, 8, 9, 10, 11, 12, 14, 16, 18, 21, 24, 27, 32, 37, 43, 50, 58, 67, 77, 88, 101, 114, 128, 143, 158, 174, 189, 203, 216, 228, 239, 246, 252, 255
// perceived intensity is basically linearly increasing
#define SQUARED 4, 4, 4, 5, 6, 7, 8, 9, 10, 12, 14, 16, 18, 21, 24, 27, 30, 33, 37, 40, 44, 48, 53, 57, 62, 67, 72, 77, 83, 88, 94, 100, 107, 113, 120, 127, 134, 141, 149, 157, 165, 173, 181, 190, 198, 207, 216, 226, 235, 245, 255
// smooth sinusoidal ramping
#define SIN_SQUARED_4 4, 4, 4, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 7, 7, 8, 9, 10, 10, 11, 13, 14, 15, 16, 18, 20, 21, 23, 25, 28, 30, 32, 35, 38, 41, 44, 47, 50, 54, 57, 61, 65, 69, 73, 77, 81, 86, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150, 156, 161, 166, 171, 176, 181, 186, 190, 195, 200, 204, 209, 213, 217, 221, 224, 228, 231, 234, 237, 240, 243, 245, 247, 249, 250, 252, 253, 254, 254, 255, 255
// smooth sinusoidal ramping
#define SIN_SQUARED 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 8, 8, 9, 10, 11, 11, 12, 14, 15, 16, 17, 19, 21, 22, 24, 26, 29, 31, 33, 36, 39, 42, 45, 48, 51, 54, 58, 62, 66, 69, 74, 78, 82, 87, 91, 96, 100, 105, 110, 115, 120, 125, 130, 135, 140, 146, 151, 156, 161, 166, 171, 176, 181, 186, 191, 195, 200, 204, 209, 213, 217, 221, 224, 228, 231, 234, 237, 240, 243, 245, 247, 249, 251, 252, 253, 254, 254, 255, 255

// select which ramping profile to use.
// store in program memory. It would use too much SRAM.
uint8_t const ramp_LUT[] PROGMEM = { SQUARED }; //SIN_SQUARED

#define ADC_CHANNEL 0x01    // MUX 01 corresponds with PB2
#define ADC_DIDR    ADC1D   // Digital input disable bit corresponding with PB2
#define ADC_PRSCL   0x06    // clk/64
#define ADC_42          185 // the ADC value we expect for 4.20 volts
#define ADC_100         185 // the ADC value for 100% full (4.2V resting)
#define ADC_75          175 // the ADC value for 75% full (4.0V resting)
#define ADC_50          164 // the ADC value for 50% full (3.8V resting)
#define ADC_25          154 // the ADC value for 25% full (3.6V resting)
#define ADC_0           139 // the ADC value for 0% full (3.3V resting)
#define ADC_LOW         123 // When do we start ramping down
#define ADC_CRIT        113 // When do we shut the light off

PROGMEM const uint8_t voltage_blinks[] = {
    ADC_0,    // 1 blink  for 0%-25%
    ADC_25,   // 2 blinks for 25%-50%
    ADC_50,   // 3 blinks for 50%-75%
    ADC_75,   // 4 blinks for 75%-100%
    ADC_100,  // 5 blinks for >100%
};



/* Rise-Fall Ramping brightness selection /\/\/\/\
 * cycle through PWM values from ramp_LUT (look up table). Traverse LUT
 * forwards, then backwards. Current PWM value is saved in noinit_lvl so
 * it is available at next startup (after a short press).
*/
void ramp()
{
    uint8_t i = 0;
    while (1){
        for (i = 0; i < sizeof(ramp_LUT); i++){
            PWM_LVL = pgm_read_byte(&(ramp_LUT[i]));
            noinit_lvl = PWM_LVL; // remember after short power off
            _delay_ms(RAMP_DELAY); //gives a period of x seconds
        }
        for (i = sizeof(ramp_LUT) - 1; i > 0; i--){
            PWM_LVL = pgm_read_byte(&(ramp_LUT[i]));
            noinit_lvl = PWM_LVL; // remember after short power off
            _delay_ms(RAMP_DELAY); //gives a period of x seconds
        }

    }
}

/* Rising Ramping brightness selection //////
 * Cycle through PWM values from ramp_LUT (look up table). Current PWM
 * value is saved in noinit_lvl so it is available at next startup
 * (after a short press)
*/
void ramp2()
{
    uint8_t i = 0;
    while (1){
        for (i = 0; i < sizeof(ramp_LUT); i++){
            PWM_LVL = pgm_read_byte(&(ramp_LUT[i]));
            noinit_lvl = PWM_LVL; // remember after short power off
            _delay_ms(RAMP_DELAY); //gives a period of x seconds
        }

        //_delay_ms(1000);
    }
}

// strobe just by changing pwm, can use this with normal pwm pin setup
static void inline pwm_strobe()
{
    while (1){
        PWM_LVL = 255;
        _delay_ms(10);
        PWM_LVL = 0;
        _delay_ms(1000);
    }
}

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
static void inline strobe2(uint16_t on, uint16_t off)
{
    while (1){
        PORTB |= _BV(STROBE_PIN); // on
        sleep_ms(on);
        PORTB &= ~_BV(STROBE_PIN); // off
        sleep_ms(off);
    }
}

inline void ADC_on() {
    ADMUX  = (1 << REFS0) | (1 << ADLAR) | ADC_CHANNEL; // 1.1v reference, left-adjust, ADC1/PB2
    DIDR0 |= (1 << ADC_DIDR);                           // disable digital input on ADC pin to reduce power consumption
    ADCSRA = (1 << ADEN ) | (1 << ADSC ) | ADC_PRSCL;   // enable, start, prescale
}

uint8_t get_voltage() {
    // Start conversion
    ADCSRA |= (1 << ADSC);
    // Wait for completion
    while (ADCSRA & (1 << ADSC));
    // See if voltage is lower than what we were looking for
    return ADCH;
}


void battcheck()
{
    uint8_t voltage, i;
    while (1){
		uint8_t blinks = 0;
		// turn off and wait one second before showing the value
		// (also, ensure voltage is measured while not under load)
		PWM_LVL = 0;
		_delay_ms(1000);
		voltage = get_voltage();
		voltage = get_voltage(); // the first one is unreliable
		// division takes too much flash space
		//voltage = (voltage-ADC_LOW) / (((ADC_42 - 15) - ADC_LOW) >> 2);
		// a table uses less space than 5 logic clauses
		for (i=0; i<sizeof(voltage_blinks); i++)
			if (voltage > pgm_read_byte(voltage_blinks + i)) blinks ++;
		// blink up to five times to show voltage
		// (~0%, ~25%, ~50%, ~75%, ~100%, >100%)
		for(i=0; i<blinks; i++) {
			PWM_LVL = 0x40;
			_delay_ms(100);
			PWM_LVL = 0;
			_delay_ms(400);
		}
		//_delay_ms(1000);  // wait at least 1 second between readouts
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
        noinit_lvl = 0;

        #ifdef  MODE_MEMORY // get mode from eeprom
        noinit_mode =  eeprom_read_byte(&MODE_P);
		noinit_lvl = eeprom_read_byte(&LVL_P);
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
    if (noinit_mode > 6) //there are 7 modes
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
            strobe();
            break;
        }
    }

    // Initialise PWM on output pin and set level to zero
    TCCR0A = PWM_TCR;
    TCCR0B = PWM_SCL;

    PWM_LVL = 0;

    uint8_t lowbatt_cnt = 0;
    uint8_t voltage;
    ADC_on();
    ACSR   |=  (1<<7); //AC off

    // keep track of the number of very short on times
    // used to decide when to go into strobe mode
    _delay_ms(25); // on for too long
    noinit_short = 0; // reset short press counter

    switch(noinit_mode){
        case 0:
        PWM_LVL = 0x05;//0xFF;
        break;
        case 1:
        PWM_LVL = 0x10;//0x40;
        break;
        case 2:
        PWM_LVL = 0x40;//0x10;
        break;
        case 3:
        PWM_LVL = 0xFF;//0x04;
        break;
        case 4:
        #ifdef MODE_MEMORY // remember mode in eeprom
        // save mode without delay, since ramp() will not return.
	    eeprom_busy_wait(); //make sure eeprom is ready
	    eeprom_write_byte(&MODE_P, noinit_mode); // save mode
	    #endif
		battcheck();
        break;
        case 5:
		pwm_strobe();// beacon actually //PWM_LVL = noinit_lvl; // use value selected by ramping function
        break;
        case 6:
        ramp(); // ramping brightness selection (no more)
        break;
    }
    
    #ifdef MODE_MEMORY // remember mode in eeprom
    eeprom_busy_wait(); //make sure eeprom is ready
    eeprom_write_byte(&MODE_P, noinit_mode); // save mode
    // only save level if it was set, to reduce writes. Not based on 
    // mode number in case mode orders change in code.
    if (noinit_lvl != 0)
    {
		eeprom_busy_wait(); //make sure eeprom is ready
	    eeprom_write_byte(&LVL_P, noinit_lvl); // save level
	}
    #endif

    while(1){
        if (ADCSRA & (1 << ADIF)) {  // if a voltage reading is ready
            voltage = get_voltage();
            // See if voltage is lower than what we were looking for
            if (voltage < ADC_LOW) ++lowbatt_cnt;
            else lowbatt_cnt = 0;
            // See if it's been low for a while, and maybe step down
            if (lowbatt_cnt == 4) PWM_LVL = 0x05;
            if (lowbatt_cnt >= 8) {
				PWM_LVL = 0; // Turn off the light
				// Power down as many components as possible
				set_sleep_mode(SLEEP_MODE_PWR_DOWN);
				sleep_mode();
			}	
            _delay_ms(1000);
            // Make sure conversion is running for next time through
            ADCSRA |= (1 << ADSC);
        }
	}
    return 0;
}