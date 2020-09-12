/*
 * main.h
 *
 * Created: 22.08.2020 20:47:55
 *  Author: Павел
 */

#pragma once

#define F_CPU 16000000UL

#include "Tables.h"

//#define DEBUG	//Prints out motor enabled/disabled events (mainly reserved for future use)
#ifdef DEBUG
	//#define DEBUG_CURRENT //Prints out calculated input, output and temporary values
	//#define DEBUG_VERBOSE	//Prints out execution location
#endif // DEBUG

// Switches timer1 prescaler to 1 = 16MHz base frequency (if enabled) or 8 = 2MHz base frequency (if disabled) and switches settings defined below
#define USE_FAST_MODEL 1
// Use table-based acceleration and input mapping (10-bit ADC values into 16-bit timer intervals), if disabled uses simple fallback implementation
#define USE_NONLINEAR_MODEL 1
// When set to 0 disables internal BCD output, leaving PORTD free for use
#define USE_BCD_DISPLAY 1
// When enabled, uses Phase & Frequency correct PWM mode (dual-slope)
#define USE_DUAL_SLOPE 1

#define DRIVE_DISPATCH_DELAY 20_ui8 //x1mS
#define BAUD_RATE 57600 //Low rates (9600, 19200) cause lags, especially during debugging!
#define DOWNSCALE_MULT (TIMER_MAX / (sizeof(acceleration_table) / sizeof(uint16_t) - 1u)) //FFC0/3FF (Use only powers of 2!)
#define TIMER_MAX 0xFFC0 //Leave room for MIN_INTERVAL addition!
#if !USE_NONLINEAR_MODEL
	#define MAX_ACCELERATION_MARGIN 4_ui8 //lowers max acceleration to 1 if current period is smaller than MAX_ACCELERATION_MARGIN * MIN_INTERVAL
#endif
#if USE_FAST_MODEL //MIN_INTERVAL has to be less than 63 (0x3F)
	#define MIN_INTERVAL 36_ui8 //4.5uS (phase & frequency correct PWM = dual slope operation, i.e. divide frequency by 2)
	#define PULSE_LENGTH 16_ui8 //2uS
#else
	#define MIN_INTERVAL 4_ui8 //5uS
	#define PULSE_LENGTH 2_ui8 //2uS
#endif

//Port C (ADC), only low frequency (interference) signals here!
#define PIN_ALM PORTC2
//Port D: Upper nibble reserved for BCD display, bits 2..3 - for addressing
#if USE_BCD_DISPLAY
	#define DISPLAY_DIGITS 4_ui8
#endif
//PORTB, Fixed: (Pulse pin is OCR1B = PB2 = 10) and led location on Arduino boards
#define PIN_PUL PORTB2
#define PIN_DIR PORTB3
#define PIN_ENA PORTB4
#define PIN_LED PORTB5

#define STARTUP_ENABLE_DEFAULT (0 << PIN_ENA)
#define MICROSTEPPING_DEFAULT 12800u
#if USE_FAST_MODEL
	#define MAX_ACCELERATION_DEFAULT 100u
#else
	#define MAX_ACCELERATION_DEFAULT 10u
#endif

namespace drive {

	typedef uint16_t interval_t; //Data type for speed (expressed as interval, not as frequency) and related units

	extern uint16_t display_value;
#if !USE_NONLINEAR_MODEL
	extern interval_t max_acceleration;
#endif
	extern bool enable; //Motor

	void init(
		uint8_t _portb_init,
		uint16_t _microstepping
	#if !USE_NONLINEAR_MODEL
		,
		interval_t _max_acceleration
	#endif
	);
	void dispatch();
	void dispatch_fast();
	void set_microstepping(uint16_t _microstepping);
	void set_speed(float _speed); //RPM
	void set_interval(interval_t _interval);
	void set_direction(bool ccw);
	void set_target(float _rotations);
	void run(float _rotations);
	void run();
	void stop();
	void reset_current();
	uint32_t get_target_steps();
	uint32_t get_current_steps();
	interval_t get_current_interval();
	interval_t get_target_interval();
	bool get_alarm();
	interval_t get_current_acceleration();

}