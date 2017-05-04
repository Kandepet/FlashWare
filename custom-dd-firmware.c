/*
 * Custom FET+7135 flashlight driver firmware based on the Toy Keeper's
 * generic firmware (tk-otc).
 * Expects a FET+1 style driver, supports two independent power channels.
 *
 * Copyright (C) 2015 Selene Scriven
 * Copyright (C) 2017 Deepak Kandepet
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *
 */

/******************** hardware-specific values **************************/
// ATTINY13
#define F_CPU 4800000UL
#define EEPSIZE 64
#define V_REF REFS0
#define BOGOMIPS 950

/******************** I/O pin and register layout ************************/
/*
 *         ATTINY13A
 *           ----
 *   Reset -|1  8|- VCC
 *     OTC -|2  7|- Voltage ADC
 *  Star 3 -|3  6|- PWM (FET)
 *     GND -|4  5|- PWM (1x7135)
 *           ----
 */

#define STAR2_PIN   PB0     // If this pin isn't used for ALT_PWM
#define STAR3_PIN   PB4     // pin 3

#define CAP_PIN     PB3     // pin 2, OTC
#define CAP_CHANNEL 0x03    // MUX 03 corresponds with PB3 (Star 4)
#define CAP_DIDR    ADC3D   // Digital input disable bit corresponding with PB3

#define PWM_PIN     PB1     // pin 6, FET PWM
#define PWM_LVL     OCR0B   // OCR0B is the output compare register for PB1
#define ALT_PWM_PIN PB0     // pin 5, 1x7135 PWM
#define ALT_PWM_LVL OCR0A   // OCR0A is the output compare register for PB0

#define VOLTAGE_PIN PB2     // pin 7, voltage ADC
#define ADC_CHANNEL 0x01    // MUX 01 corresponds with PB2
#define ADC_DIDR    ADC1D   // Digital input disable bit corresponding with PB2
#define ADC_PRSCL   0x06    // clk/64

/******************* PWM *****************************/
//#define FAST 0x23           // fast PWM channel 1 only
//#define PHASE 0x21          // phase-correct PWM channel 1 only
#define FAST 0xA3           // fast PWM both channels
#define PHASE 0xA1          // phase-correct PWM both channels

// output to use for blinks on battery check mode (primary PWM level, alt PWM level)
#define BLINK_BRIGHTNESS    0,20

// Mode group
#define NUM_MODES           3
// PWM levels for the FET
#define FET_LEVELS             0,7,137 // Turbo mode (255) is a special mode defined below

// PWM levels for the small circuit (1x7135)
#define LM7135_LEVELS             3,110,255

// PWM speed for each mode
#define PWM_MODES           PHASE,FAST,FAST

#define NUM_SPECIAL_MODES          4
#define SPECIALMODES         TURBO,STROBE,BIKING_STROBE,BATTCHECK
#define SPECIALMODES_PWM_MODES     PHASE,PHASE,PHASE,PHASE
#define SPECIALMODES_LM7135_LEVELS     0,0,0,0   // Zeroes, same length as NUM_SPECIAL_MODES

#define TURBO     255       // Convenience code for turbo mode (has to be unsigned byte)
#define BATTCHECK 254       // Convenience code for battery check mode
#define STROBE    253       // Convenience code for strobe mode
#define BIKING_STROBE 252   // Convenience code for biking strobe mode

#define BLINK_SPEED 500

// How many timer ticks before before dropping down.
// Each timer tick is 500ms, so "60" would be a 30-second stepdown.
// Max value of 255 unless you change "ticks"
#define TURBO_TIMEOUT 90 // only for testing

/********************** Voltage ADC calibration **************************/
// Calibration values for voltage and OTC
// These values were measured using RMM's FET+7135.
// See battcheck/readings.txt for reference values.
// These are the ADC values we expect for specific voltages
#define ADC_44     194
#define ADC_43     189
#define ADC_42     184
#define ADC_41     178
#define ADC_40     173
#define ADC_39     168
#define ADC_38     163
#define ADC_37     158
#define ADC_36     152
#define ADC_35     147
#define ADC_34     142
#define ADC_33     137
#define ADC_32     131
#define ADC_31     126
#define ADC_30     121
#define ADC_29     116
#define ADC_28     111
#define ADC_27     105
#define ADC_26     100
#define ADC_25     95
#define ADC_24     90
#define ADC_23     84
#define ADC_22     79
#define ADC_21     74
#define ADC_20     69

#define ADC_100p   ADC_42  // the ADC value for 100% full (resting)
#define ADC_75p    ADC_40  // the ADC value for 75% full (resting)
#define ADC_50p    ADC_38  // the ADC value for 50% full (resting)
#define ADC_25p    ADC_35  // the ADC value for 25% full (resting)
#define ADC_0p     ADC_30  // the ADC value for 0% full (resting)
#define ADC_LOW    ADC_30  // When do we start ramping down
#define ADC_CRIT   ADC_27  // When do we shut the light off


/********************** Offtime capacitor calibration ********************/
// Values are between 1 and 255, and can be measured with offtime-cap.c
// See battcheck/otc-readings.txt for reference values.
// These #defines are the edge boundaries, not the center of the target.
#ifdef USE_MEDIUM_PRESS
// The OTC value 0.5s after being disconnected from power
// (anything higher than this is a "short press")
#define CAP_SHORT           190
// The OTC value 1.5s after being disconnected from power
// Between CAP_MED and CAP_SHORT is a "medium press"
#define CAP_MED             94
// Below CAP_MED is a long press
#else
// The OTC value 1.0s after being disconnected from power
// Anything higher than this is a short press, lower is a long press
#define CAP_SHORT           115
#endif


// Ignore a spurious warning, we did the cast on purpose
#pragma GCC diagnostic ignored "-Wint-to-pointer-cast"

#include <avr/pgmspace.h>
#include <avr/eeprom.h>
#include <avr/sleep.h>
#include <util/delay_basic.h>

// Having own _delay_ms() saves some bytes AND adds possibility to use variables as input
void _delay_ms(uint16_t n)
{
    while(n-- > 0) _delay_loop_2(BOGOMIPS);
}

void _delay_s()  // because it saves a bit of ROM space to do it this way
{
    _delay_ms(1000);
}

inline void ADC_on() {
    // disable digital input on ADC pin to reduce power consumption
    DIDR0 |= (1 << ADC_DIDR);
    // 1.1v reference, left-adjust, ADC1/PB2
    ADMUX  = (1 << V_REF) | (1 << ADLAR) | ADC_CHANNEL;
    // enable, start, prescale
    ADCSRA = (1 << ADEN ) | (1 << ADSC ) | ADC_PRSCL;
}

uint8_t get_voltage() {
    // Start conversion
    ADCSRA |= (1 << ADSC);
    // Wait for completion
    while (ADCSRA & (1 << ADSC));
    // Send back the result
    return ADCH;
}

/* Battery voltage to blinks mapping for battery check.
 * We use 18350/18650 with voltage of 3.7v.
 *  #   186 - 4.00V - 183
 *  #   176 - 3.80V - 173
 *  #   169 - 3.64V
 *  #   162 - 3.50V - 160
 *  #   160 - 3.45V
 *  #   138 - 3.00V - 137
 *  #   134 - 2.90V - 132
 *  #   129 - 2.80V - 127
 *  #   124 - 2.70V - 123
 */
PROGMEM const uint8_t voltage_blinks[] = {
               // 0 blinks for less than 1%
    ADC_30,    // 1 blink  for 1%-12.5% (~2.7v)
    ADC_33,    // 2 blinks for 12.5%-25% (~2.8v)
    ADC_35,    // 3 blinks for 25%-37.5% (~2.9v)
    ADC_37,    // 4 blinks for 37.5%-50% (~3.0v)
    ADC_38,    // 5 blinks for 50%-62.5% (~3.45v)
    ADC_39,    // 6 blinks for 62.5%-75% (~3.5v)
    ADC_40,    // 7 blinks for 75%-87.5% (~3.64v)
    ADC_41,    // 8 blinks for 87.5%-100% (~3.80v)
    ADC_42,    // 9 blinks for >100% (~4.0v)
    255,       // Ceiling, don't remove  (10 blinks means "error")
};

inline uint8_t battcheck() {
    // Return an int, number of "blinks", for approximate battery charge
    // Uses the table above for return values
    uint8_t i, voltage;
    voltage = get_voltage();
    // figure out how many times to blink
    for (i=0;
         voltage > pgm_read_byte(voltage_blinks + i);
         i ++) {}
    return i;
}


// Global config / state variables
uint8_t eepos = 0;         // EEPROM read/write position
uint8_t memory = 1;        // == 1 remember last mode,
                           // == 0 start at moonlight mode
uint8_t mode_idx = 0;      // current or last-used mode number

// number of regular non-special modes in mode group
#define SOLID_MODES NUM_MODES
// total length of mode group's array
#define MODE_COUNT SOLID_MODES+NUM_SPECIAL_MODES

// Modes (gets set when the light starts up based on saved config values)
// NOTE: SPECIALMODES only has special codes, the actual level for each special
// mode is set in code.
PROGMEM const uint8_t fet_levels[] = { FET_LEVELS, SPECIALMODES };
PROGMEM const uint8_t lm7135_levels[] = { LM7135_LEVELS, SPECIALMODES_LM7135_LEVELS };
PROGMEM const uint8_t pwm_modes[] = { PWM_MODES, SPECIALMODES_PWM_MODES };

void save_state() {  // central method for writing (with wear leveling)
    // a single 16-bit write uses less ROM space than two 8-bit writes
    uint8_t eep;
    uint8_t oldpos=eepos;

    eepos = (eepos+1) & (EEPSIZE-1);  // wear leveling, use next cell

    eep = mode_idx;
    eeprom_write_byte((uint8_t *)(eepos), eep);      // save current state
    eeprom_write_byte((uint8_t *)(oldpos), 0xff);    // erase old state
}

void restore_state() {
    uint8_t eep;
    // find the config data
    for(eepos=0; eepos<EEPSIZE; eepos++) {
        eep = eeprom_read_byte((const uint8_t *)eepos);
        if (eep != 0xff) break;
    }
    // unpack the config data
    if (eepos < EEPSIZE) {
        mode_idx = eep;
    }
    // unnecessary, save_state handles wrap-around
    // (and we don't really care about it skipping cell 0 once in a while)
    //else eepos=0;
}

inline void next_mode() {
    mode_idx += 1;
    //if (mode_idx >= SOLID_MODES) {
    if (mode_idx >= MODE_COUNT) {
        // Wrap around
        mode_idx = 0;
    }
}

inline void prev_mode() {
    if (mode_idx > 0) {
        mode_idx -= 1;
    } else {
        // Otherwise, wrap around (this allows entering special modes)
        mode_idx = MODE_COUNT - 1;
    }
}

void set_output(uint8_t pwm1, uint8_t pwm2) {
    // Need PHASE to properly turn off the light
    if ((pwm1==0) && (pwm2==0)) {
        TCCR0A = PHASE;
    }
    PWM_LVL = pwm1;
    ALT_PWM_LVL = pwm2;
}

void set_mode(uint8_t mode) {
    TCCR0A = pgm_read_byte(pwm_modes + mode);
    set_output(pgm_read_byte(fet_levels + mode), pgm_read_byte(lm7135_levels + mode));
}

void blink(uint8_t val)
{
    for (; val>0; val--)
    {
        set_output(BLINK_BRIGHTNESS);
        _delay_ms(BLINK_SPEED / 5);
        set_output(0,0);
        _delay_ms(BLINK_SPEED * 4 / 5);
    }
}

int main(void)
{
    uint8_t cap_val;

    // Read the off-time cap *first* to get the most accurate reading
    // Start up ADC for capacitor pin
    DIDR0 |= (1 << CAP_DIDR);                           // disable digital input on ADC pin to reduce power consumption
    ADMUX  = (1 << V_REF) | (1 << ADLAR) | CAP_CHANNEL; // 1.1v reference, left-adjust, ADC3/PB3
    ADCSRA = (1 << ADEN ) | (1 << ADSC ) | ADC_PRSCL;   // enable, start, prescale

    // Wait for completion
    while (ADCSRA & (1 << ADSC));

    // Start again as datasheet says first result is unreliable
    ADCSRA |= (1 << ADSC);

    // Wait for completion
    while (ADCSRA & (1 << ADSC));

    cap_val = ADCH; // save this for later

    // All ports default to input, but turn pull-up resistors on for the stars (not the ADC input!  Made that mistake already)
    // only one star, because one is used for PWM channel 2
    // and the other is used for the off-time capacitor
    PORTB = (1 << STAR3_PIN);

    // Set PWM pin to output
    DDRB |= (1 << PWM_PIN);     // enable main channel
    DDRB |= (1 << ALT_PWM_PIN); // enable second channel

    // Set timer to do PWM for correct output pin and set prescaler timing
    //TCCR0A = 0x23; // phase corrected PWM is 0x21 for PB1, fast-PWM is 0x23
    //TCCR0B = 0x01; // pre-scaler for timer (1 => 1, 2 => 8, 3 => 64...)
    TCCR0A = PHASE;
    // Set timer to do PWM for correct output pin and set prescaler timing
    TCCR0B = 0x01; // pre-scaler for timer (1 => 1, 2 => 8, 3 => 64...)

    // Read config values and saved state
    restore_state();


    if (cap_val > CAP_SHORT) {
        // Indicates they did a short press, go to the next mode
        next_mode(); // Will handle wrap arounds
#ifdef USE_MEDIUM_PRESS
    } else if (cap_val > CAP_MED) {
        // User did a medium press, go back one mode
        prev_mode(); // Will handle "negative" modes and wrap-arounds
#endif
    } else {
        // Long press, keep the same mode
        // ... or reset to the first mode
        if (! memory) {
            // Reset to the first mode
            mode_idx = 0;
        }
    }
    save_state();

    // Turn off ADC
    //ADC_off();

    // Charge up the capacitor by setting CAP_PIN to output
    DDRB  |= (1 << CAP_PIN);    // Output
    PORTB |= (1 << CAP_PIN);    // High

    // Turn features on or off as needed
    ADC_on();
    //ACSR   |=  (1<<7); //AC off

    uint8_t output;
    uint8_t ticks = 0;

    // Voltage Mon
    uint8_t lowbatt_cnt = 0;
    uint8_t i = 0;
    uint8_t voltage;
    // Make sure voltage reading is running for later
    ADCSRA |= (1 << ADSC);

    while(1) {
        output = pgm_read_byte(fet_levels + mode_idx);
        if (output == STROBE) {
            // 10Hz tactical strobe
            set_output(255,0);
            _delay_ms(50);
            set_output(0,0);
            _delay_ms(50);
        }
        else if (output == BIKING_STROBE) {
            // 2-level stutter beacon for biking and such
            // normal version
            for(i=0;i<4;i++) {
                set_output(255,0);
                _delay_ms(5);
                set_output(0,255);
                _delay_ms(65);
            }
            _delay_ms(720);
        }

        // Check Battery status
        else if (output == BATTCHECK) {
            blink(battcheck());
            // wait between readouts
            _delay_s(); _delay_s();
        }

        else {  // Regular non-special solid mode
            set_mode(mode_idx);

            // Track if and for how long we are in turbo mode and drop down smartly.
            // Do some magic here to handle turbo step-down
            //if (ticks < 255) ticks++;  // don't roll over
            ticks ++;  // actually, we don't care about roll-over prevention
            if ((ticks > TURBO_TIMEOUT)
                    && (output == TURBO)) {
                //mode_idx = SOLID_MODES - 2; // step down to second-highest mode
                //Step down to whatever is the previous mode
                prev_mode();
                set_mode(mode_idx);
                save_state();
            }
            // Otherwise, just sleep.
            _delay_ms(500);
        }

        // Voltage Monitoring
        if (ADCSRA & (1 << ADIF)) {  // if a voltage reading is ready
            voltage = ADCH; // get_voltage();
            // See if voltage is lower than what we were looking for
            //if (voltage < ((mode_idx <= 1) ? ADC_CRIT : ADC_LOW)) {
            if (voltage < ADC_LOW) {
                lowbatt_cnt ++;
            } else {
                lowbatt_cnt = 0;
            }
            // See if it's been low for a while, and maybe step down
            if (lowbatt_cnt >= 8) {
                // DEBUG: blink on step-down:
                //set_output(0,0);  _delay_ms(100);
                i = mode_idx; // save space by not accessing mode_idx more than necessary
                // properly track special vs normal modes
                if (i >= SOLID_MODES) {
                    // step down from blinky modes to medium
                    i = 2;
                } else if (i > 0) {
                    // step down from solid modes one at a time
                    i -= 1;
                } else { // Already at the lowest mode
                    i = 0;
                    // Turn off the light
                    set_output(0,0);
                    // Power down as many components as possible
                    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
                    sleep_mode();
                }
                set_mode(i);
                mode_idx = i;
                save_state();
                lowbatt_cnt = 0;
                // Wait at least 2 seconds before lowering the level again
                _delay_ms(250);  // this will interrupt blinky modes
            }

            // Make sure conversion is running for next time through
            ADCSRA |= (1 << ADSC);
        }
        //sleep_mode();  // incompatible with blinky modes
    }

    //return 0; // Standard Return Code
}
