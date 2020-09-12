/*
 * StepperSpeedController.cpp

 This is a simplified alternative to AccelStepper library. The latter allows adjusting control parameters in software,
 and therefore has to perform time-consuming computations. That's why AccelStepper's max step rate on 16MHz ATmega is around 10kHz.
 This application uses hardware timer to increase step rate limit to 200kHz (common motor driver bandwidth).
 It uses precalculated acceleration and control curves (see tables.h).
 This allows to decrease time of response to external controls (a potentiometer etc) and gradually implement new features
 without fear to run out of main loop time. The profiles can be modified only by editing the source and reflashing the uC.
 Unfortunately, this is a necessary trade-of that can be overcome only by switching to more powerful 32-bit architectures (XMEGA, STM32 or ARM).

 TODO:
 - add more UART features (framework already in place), for example profile switching.
 - fix deceleration

 *
 * Created: 19.08.2020 20:57:20
 * Author : Павел
 */

#define F_CPU 16000000UL

#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <util/delay.h>
#include <stdlib.h>
#include <stdio.h>
#include <limits.h>
#include "StepperSpeedController.h"
#include "uart.h"
#include "ProgmemLate.h"
#include "MyFunctions.h"
#include "Tables.h"

#define PIN_BTN PORTC1
#define PIN_KNOB PORTC0
#define PIN_DBG PORTC3
#define EEPROM_CHECK_VALUE 0x29
#define AVERAGING_WINDOW_SIZE 8_ui8
#define DEBOUNCING_DELAY 40_ui8 //x DRIVE_DISPATCH_DELAY

EEMEM uint8_t eeprom_check = EEPROM_CHECK_VALUE;
EEMEM uint8_t startup_enable = STARTUP_ENABLE_DEFAULT;
EEMEM uint16_t microstepping_eep = MICROSTEPPING_DEFAULT;
EEMEM drive::interval_t max_acceleration_eep = MAX_ACCELERATION_DEFAULT; //Left here regardless of USE_NONLINEAR_MODEL to preserve EEPROM layout

volatile uint8_t timer_counter;

static void load_uart_data(char* ptr);
static void print_eeprom(char* buf);
static void process_cmd(uint8_t c);
static uint16_t read_adc();
static void poll();

ISR(TIMER0_COMPA_vect, ISR_NAKED)
{
	asm volatile( //Optimized ABI (only SREG and __tmp_reg__ are clobbered)
		"push __tmp_reg__ \n\t"
		"in __tmp_reg__, __SREG__ \n\t"
		"push __tmp_reg__ \n\t"
	);
	sei(); //ISR_NOBLOCK alternative (can't pass second ISR option for some reason on AVR-GCC 5.4.0)
	drive::dispatch_fast(); //Uses only callee-saved registers, safe for ISR_NAKED
	asm volatile( //Increment timer_counter (C would've used upper registers instead of push-ed __tmp_reg__ for some reason, as it always does)
		"lds	__tmp_reg__, %0 \n\t"
		"inc	__tmp_reg__ \n\t"
		"sts	%0, __tmp_reg__ \n\t"
		:
		: "m" (timer_counter)
		:
	);
#ifdef DEBUG
	asm volatile( //Toggle debug pin (to measure timer frequency), can't use registers except __tmp_reg__ and __SREG__
		"clt \n\t"
		"sbic %i0, %1 \n\t"
		"set \n\t"
		"cbi %i0, %1 \n\t"
		"brts .+2 \n\t"
		"sbi %i0, %1 \n\t"
		:
		: "n" (&(PORTC)), "I" (PIN_DBG)
		:
	);
#endif // DEBUG
	asm volatile(
		"pop __tmp_reg__ \n\t"
		"out __SREG__, __tmp_reg__ \n\t"
		"pop __tmp_reg__ \n\t"
	);
	reti();
	__builtin_unreachable();
}

void setup(void)
{
	cli();
	// ADC initialization
	// ADC Clock frequency: 125,000 kHz
	// ADC Voltage Reference: AVCC pin
	// ADC Auto Trigger Source: Free Running
	//
	// the AD conversion result are used
	// Digital input buffers on ADC0: Off, ADC1: On, ADC2: On, ADC3: On
	// ADC4: On, ADC5: On
	DIDR0 = 0x01;
	ADMUX = (0 << REFS1) | (1 << REFS0) | (0 << ADLAR) | PIN_KNOB; //Switched to ADC0, Use ADCH to read the result
	ADCSRA = (1 << ADEN) | (1 << ADATE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
	ADCSRB = 0;
	ADCSRA |= (1 << ADSC); //Discard first conversion
	//Setup 1mS utility timer
	TCNT0  = 0; // initialize counter value to 0
	// set compare match register for 1000 Hz increments
	OCR0A = 249; // = 16000000 / (256 * 250) - 1 (must be <256)
	// turn on CTC mode
	TCCR0A = (1 << WGM01); //Beware: http://www.8bit-era.cz/ calculator makes mistakes here!!!!!!!!!!!!!!!!!!!
	// Set CS02, CS01 and CS00 bits for 64 prescaler
	TCCR0B = (0 << CS02) | (1 << CS01) | (1 << CS00);
	// enable timer compare interrupt
	TIMSK0 = (1 << OCIE0A);

	if (eeprom_read_byte(&eeprom_check) != EEPROM_CHECK_VALUE)
	{
		eeprom_write_byte(&eeprom_check, EEPROM_CHECK_VALUE);
		eeprom_update_byte(&startup_enable, STARTUP_ENABLE_DEFAULT);
		eeprom_update_word(&microstepping_eep, MICROSTEPPING_DEFAULT);
		static_assert(sizeof(drive::interval_t) == sizeof(uint16_t), "Update EEPROM function to support new data type.");
		eeprom_update_word(&max_acceleration_eep, MAX_ACCELERATION_DEFAULT);
	}
#ifdef DEBUG
	DDRC |= BV8(PIN_DBG);
#endif // DEBUG
	PORTC |= BV8(PIN_BTN);
	uart_init(UART_BAUD_SELECT(BAUD_RATE, F_CPU));
	drive::init(
		eeprom_read_byte(&startup_enable),
		eeprom_read_word(&microstepping_eep)
#if !USE_NONLINEAR_MODEL
		,
		eeprom_read_word(&max_acceleration_eep)
#endif
	);

	sei();
	uart_puts_P("Setup OK.\n");
}

int main(void)
{
	setup();

	bool uart_enabled = true;
	uint8_t dispatch_last = 0;
	while (1)
	{
		if (static_cast<uint8_t>(timer_counter - dispatch_last) > DRIVE_DISPATCH_DELAY)
		{
			dispatch_last = timer_counter;
			//Input devices
			poll();
			//Dispatch the library
			drive::dispatch();
		}
		//PC commands
		if (drive::enable)
		{
			if (uart_enabled)
			{
#ifndef DEBUG
				uart_disable();
#endif
				uart_enabled = false;
			}
		}
		else
		{
			if (uart_enabled)
			{
				uint16_t temp = uart_getc();
				_delay_us(20);
				if ((temp >> CHAR_BIT) == 0) process_cmd(static_cast<uint8_t>(temp));
			}
			else
			{
				uart_init(UART_BAUD_SELECT(BAUD_RATE, F_CPU));
				uart_enabled = true;
			}
		}
	}
}

void poll()
{
	static uint8_t averaging_index;
	static uint16_t averaging_window[AVERAGING_WINDOW_SIZE];
	static uint8_t debounce;
	//Run button
	if (debounce > 0) --debounce;
	else if (!(PINC & BV8(PIN_BTN)))
	{
		debounce = DEBOUNCING_DELAY;
		drive::reset_current();
		drive::enable = !drive::enable;
	}
	//Speed knob
	averaging_window[averaging_index++] = read_adc();
	if (averaging_index == AVERAGING_WINDOW_SIZE) averaging_index = 0;
	uint16_t ti = 0;
	for (uint8_t i = 0; i < AVERAGING_WINDOW_SIZE; ++i)
	{
		ti += averaging_window[i];
	}
#ifdef DEBUG_CURRENT
	char dbg_buf[8];
	uart_puts(itoa(ti, dbg_buf, 10));
	uart_putc('\n');
#endif // DEBUG_CURRENT
	ti = (ti + AVERAGING_WINDOW_SIZE / 2_ui8) / AVERAGING_WINDOW_SIZE;
	drive::set_interval(pgm_read_word(&(mapping_table[ti])) + MIN_INTERVAL);
	//TODO: position UI
}

uint16_t read_adc()
{
#ifdef DEBUG_VERBOSE
	uart_puts("ADC\n");
#endif // DEBUG
	// Start the AD conversion
	ADCSRA |= BV8(ADSC);
	// Wait for the AD conversion to complete
	while ((ADCSRA & BV8(ADSC)) == 0);
	uint8_t low = ADCL;
	return (ADCH << 8) | low;
}

#pragma region UART

void process_cmd(uint8_t c)
{
#ifdef DEBUG_VERBOSE
	uart_puts("Processing PC\n");
#endif // DEBUG
	char buf[16];
	switch (c)
	{
		case 'T':
		{
			load_uart_data(buf);
			float res = decodeFloat(buf, sizeof(buf));
			drive::set_target(res);
			uart_puts(dtostrf(res, 1, 3, buf));
			uart_putc('\n');
			break;
		}
		case 'R':
		{
			uart_puts("OK\n");
			_delay_us(50);
			drive::run();
			break;
		}
#if !USE_NONLINEAR_MODEL
		case 'A':
		{
			load_uart_data(buf);
			drive::max_acceleration = static_cast<uint8_t>(decodeInt(buf));
			eeprom_update_word(&max_acceleration_eep, drive::max_acceleration);
			print_eeprom(buf);
			break;
		}
#endif
		case 'M':
		{
			load_uart_data(buf);
			uint16_t temp = static_cast<uint16_t>(atol(buf));
			eeprom_update_word(&microstepping_eep, temp);
			drive::set_microstepping(temp);
			print_eeprom(buf);
			break;
		}
		case 'S':
		{
			load_uart_data(buf);
			uint8_t temp = 0;
			bitToInt(&temp, buf, CHAR_BIT);
			eeprom_update_byte(&startup_enable, temp);
			print_eeprom(buf);
			break;
		}
		case 'I':
			print_eeprom(buf);
			uart_puts(drive::enable ? "ENabled" : "DISabled");
			if (drive::get_alarm()) uart_puts("\nAlarm!");
			uart_puts("\nTI=");
			uart_puts(itoa(drive::get_target_interval(), buf, 16));
			uart_puts("h\nCI=");
			uart_puts(itoa(drive::get_current_interval(), buf, 16));
			uart_puts("h\nTS=");
			uart_puts(ltoa(drive::get_target_steps(), buf, 16));
			uart_puts("h\nCS=");
			uart_puts(ltoa(drive::get_current_steps(), buf, 16));
			uart_puts("h\nCA=");
			uart_puts(itoa(drive::get_current_acceleration(), buf, 16));
			uart_puts("h\n");
			break;
		default:
			uart_puts_P("Unknown command\n");
			break;
	}
}

void load_uart_data(char* ptr)
{
	uint16_t bit;
	_delay_us(200);
	while (((bit = uart_getc()) >> CHAR_BIT) == 0)
	{
		*ptr++ = static_cast<char>(bit & 0xFF);
		_delay_us(200);
	}
	*ptr = '\0';
}

void print_eeprom(char* buf)
{
#if !USE_NONLINEAR_MODEL
	uart_puts("MA=");
	uart_puts(itoa(drive::max_acceleration, buf, 10));
	uart_putc('\n');
#endif
	uart_puts("SE=");
	uart_puts(itoa(eeprom_read_byte(&startup_enable), buf, 2));
	uart_puts("b\nM=");
	uart_puts(itoa(eeprom_read_word(&microstepping_eep), buf, 16));
	uart_puts("h\n");
}

#pragma endregion UART