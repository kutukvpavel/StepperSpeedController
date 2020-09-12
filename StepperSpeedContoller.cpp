/*
 * StepperSpeedContoller.cpp
 *
 * Created: 22.08.2020 20:53:27
 *  Author: Павел
 */

#include <math.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <limits.h>
#include "MyFunctions.h"
#include "ProgmemLate.h"
#include "StepperSpeedController.h"

#if defined(DEBUG) || defined(DEBUG_CURRENT) || defined(DEBUG_VERBOSE)
	#include "uart.h"
#endif

using namespace drive;

//Public:
uint16_t drive::display_value;
#if !USE_NONLINEAR_MODEL
	interval_t drive::max_acceleration;
#endif
bool drive::enable; //Motor

//Private:
register uint8_t drive_steps_high asm("r5");
register uint8_t drive_steps_low asm("r4");
register uint8_t drive_allow_change asm("r3");
register uint8_t drive_display_count asm("r2"); //Used as a constant non-zero register, and as total 7-segment digit count
volatile uint32_t current_steps;
uint32_t target_step_limit;
uint32_t microstepping_precalculated;
uint16_t microstepping;
interval_t target_interval;
interval_t current_interval; //Timer period (in ticks)!
interval_t current_acceleration;
#if USE_BCD_DISPLAY
	uint8_t display_value_nibbles[DISPLAY_DIGITS];
	register uint8_t display_index asm("r6");
#endif

void calculate();
void process_enabling();
void enable_pwm();
void disable_pwm();

#pragma region ISRs

//1mS timer: <1% CPU (intended to be used with TIMER0 interrupt)
void drive::dispatch_fast()
{
	asm volatile("push __zero_reg__");
#if USE_BCD_DISPLAY
	asm volatile(
		"cp r6, r1 \n\t" //If display_index != 0
		"brne .+2 \n\t" //Then skip reset to drive_display_count
		"mov r6, r2 \n\t"
		"dec r6 \n\t" //whatever happens, decrement display_index
	);
	asm volatile( //C would've done this using 4 registers instead of 2 (hence 8 more cycles for push/pop)
		"add r28, r6 \n\t" //Add current display_index to the base pointer (C will generate the base pointer address code due to operand declaration)
		"adc r29, __zero_reg__ \n\t"	//Add carry to the high byte
		"ld __tmp_reg__, %a0 \n\t"	//Load current digit value
		"mov r28, r6 \n\t" //Load display_index bits
		"lsl r28 \n\t"	//Shift to the left twice
		"lsl r28 \n\t"
		"or __tmp_reg__, r28 \n\t" //Combine index bits with digit bits
		"in r28, %i1 \n\t" //Load current PORTB bits
		"andi r28, 0x03 \n\t" //Preserve only 2 UART bits
		"or __tmp_reg__, r28 \n\t" //Combine with new display output
		"out %i1, __tmp_reg__ \n\t" //Write to the port
		:
		: "y" (display_value_nibbles), "n" (&PORTD) //y pointer register, because it's callee-saved!
		:
	);
#endif
	asm volatile(
		"lds __tmp_reg__, (current_steps) \n\t" //drive_steps_... is a 16-bit registered buffer for 32-bit current_steps RAM value, need it to reduce instructions in TIMER1_ovf_vect ISR
		"lds __zero_reg__, (current_steps + 0x01) \n\t" //Can use zero_reg inside a non-naked interrupt
		"lds r28, (current_steps + 0x02) \n\t"
		"lds r29, (current_steps + 0x03) \n\t"
		"cli \n\t" //The next operation has to be atomic (it contains certainly less than 200 cycles, so no problem with losing steps)
		"add __tmp_reg__, %3 \n\t" //Add the registered buffer to the RAM value
		"adc __zero_reg__, %4 \n\t"
		"clz \n\t" //Clear zero flag in case next increment will be skipped (no carry)
		"brcc .+2 \n\t" //Carry without zero_reg
		"inc r28 \n\t" //Sets zero flag on overflow (carry)
		"brne .+2 \n\t" //Tests zero flag (branch if not set)
		"inc r29 \n\t"
		"clr %0 \n\t"
		"clr %1 \n\t"
		"sei \n\t"
		"sts %6, __tmp_reg__ \n\t"
		"sts (%6 + 0x01), __zero_reg__ \n\t"
		"sts (%6 + 0x02), r28 \n\t"
		"sts (%6 + 0x03), r29 \n\t"
		"lds r16, %5 \n\t"
		"cp __tmp_reg__, r16 \n\t"
		"lds r16, (%5 + 0x01) \n\t"
		"cpc __zero_reg__, r16 \n\t"
		"lds r16, (%5 + 0x02) \n\t"
		"cpc r28, r16 \n\t"
		"lds r16, (%5 + 0x03) \n\t"
		"cpc r29, r16 \n\t"
		"brcs .+6 \n\t"
		"clr __zero_reg__ \n\t"
		"sts %2, __zero_reg__ \n\t"
		: "=r" (drive_steps_low), "=r" (drive_steps_high)
		: "m" (drive::enable), "0" (drive_steps_low), "1" (drive_steps_high), "m" (target_step_limit), "m" (current_steps)
		: "r28", "r29", "r16", "memory" //Callee-saved
	);
	asm volatile("pop __zero_reg__");
}

//High-frequency optimized interrupt! Worst-case: takes 10 cycles each 5uS (200 cycles) = 5% CPU
ISR(TIMER1_OVF_vect, ISR_NAKED) //It has higher priority (vector 13) than TIMER0! This is intended.
{
	asm volatile(
		"in r3, __SREG__ \n\t" //Use r3 as a buffer for SREG
		"inc r4 \n\t" //Increment low byte of step counter
		"brne .+2 \n\t" //If the result is 0 (overflow) [can't use adc r5, __zero_reg__ inside a naked interrupt]
		"inc r5 \n\t" //Then carry to the high-byte
		"out __SREG__, r3 \n\t"
		"mov r3, r2 \n\t" //Set drive_allow_change to true (>0)
	);
	reti();
	__builtin_unreachable();
}

#pragma endregion ISRs

void drive::init(
	uint8_t _portb_init,
	uint16_t _microstepping
#if !USE_NONLINEAR_MODEL
	,
	interval_t _max_acceleration
#endif
)
{
	//load parameters
#if !USE_NONLINEAR_MODEL
	max_acceleration = _max_acceleration;
#endif
	set_microstepping(_microstepping);
	current_interval = TIMER_MAX;
	target_interval = TIMER_MAX;

	//initialize memory
	drive_display_count = DISPLAY_DIGITS;

	//Setup pins
#if USE_BCD_DISPLAY
	DDRD |= 0xFC; //upper nibble + 2 lower bits are reserved for BCD display
#endif
	DDRB |= (1 << PIN_PUL) | (1 << PIN_LED) | (1 << PIN_ENA) | (1 << PIN_DIR); //OCRB1 output
	PORTB |= _portb_init;
	PORTC |= (1 << PIN_ALM);

	//Setup 200kHz interrupt timer
	TCNT1  = 0; // initialize counter value to 0
#if USE_DUAL_SLOPE
	// OCR1A is used as TOP (initialize to max possible interval)
	OCR1AH = 0xFF;
	OCR1AL = 0xFF; //High byte MUST be written first, initializing to lowest possible frequency
	disable_pwm();
	// turn on Phase and Frequency Correct PWM (TOP = OCR1A)
	TCCR1A = (1 << COM1B1) | (0 << COM1B0) | (0 << WGM11) | (1 << WGM10);
	TCCR1B = (1 << WGM13) | (0 << WGM12);
#else
	#error "Single-slope not implemented yet!"
#endif
#if USE_FAST_MODEL
// Set CS12, CS11 and CS10 bits for prescaler 1
	TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10); // |= !!!
#else
// Set CS12, CS11 and CS10 bits for prescaler 8
	TCCR1B |= (0 << CS12) | (1 << CS11) | (0 << CS10); // |= !!!
#endif
	// enable timer overflow interrupt
	TIMSK1 = (1 << TOIE1);

	//Setup UART
#ifdef UART_H
	uart_init(UART_BAUD_SELECT(BAUD_RATE, F_CPU));
#endif
#ifdef DEBUG
	char dbg_buf[16];
	uart_puts(ltoa(microstepping_precalculated, dbg_buf, 10));
	uart_puts_P("\nStepper speed controller initialized.\n");
#endif // DEBUG
}

void drive::dispatch()
{
#ifdef DEBUG_VERBOSE
	uart_puts("Starting dispatch\n");
#endif
	//Update speed value
#if USE_NONLINEAR_MODEL
	static_assert(sizeof(interval_t) == sizeof(uint16_t), "Modify PGM code to support new speed value type!");
#endif
	calculate();
	asm volatile(
		"mov r3, __zero_reg__ \n\t" //Wait for next Timer1 overflow: set drive_allow_change to false
		"cp r3, r2 \n\t"	//Busy-wait for drive_allow_change = true
		"brne .-4 \n\t"
	); //Then write new OCR1, while TCNT1 is still small
	static_assert(sizeof(uint8_t) == sizeof(interval_t) || sizeof(uint16_t) == sizeof(interval_t),
		"Modify the code (OCR1A) to support new speed value type!");
	OCR1AH = static_cast<uint8_t>(current_interval >> CHAR_BIT);
	OCR1AL = static_cast<uint8_t>(current_interval & 0xFF); //Order!
	process_enabling();
}

#pragma region Functions

void calculate() //~40uS
{
#ifdef DEBUG_VERBOSE
	uart_puts("Calc\n");
#endif // DEBUG
#if USE_NONLINEAR_MODEL
	//static_assert(DOWNSCALE_MULT % 2 == 0, "Use powers of 2 for UPSCALE_MULT to avoid long loop-based division operations!");
#endif
	interval_t a =
	#if USE_NONLINEAR_MODEL
		pgm_read_word(&(acceleration_table[current_interval / DOWNSCALE_MULT])) //Division is compiled into bit shift (powers of 2 used!)
	#else
		((current_interval < (MAX_ACCELERATION_MARGIN * MIN_INTERVAL)) ? 1 : max_acceleration)
	#endif
	;
	current_acceleration = a;
#ifdef DEBUG_CURRENT
{
	char dbg_buf[8];
	uart_puts("A=");
	uart_puts(itoa(a, dbg_buf, 10));
	uart_putc('\n');
}
#endif // DEBUG
	interval_t target = enable ? target_interval : TIMER_MAX;
	if (target < current_interval)
	{
		if ((current_interval - target) > a) current_interval -= a;
		else current_interval = target;
	}
	else
	{
		if ((target - current_interval) > a) current_interval += a;
		else current_interval = target;
	}
	uint32_t temp = microstepping_precalculated / (enable ? current_interval : target_interval); //The longest operation, but there's no way to optimize it...
	display_value = static_cast<uint16_t>(temp);
#if USE_BCD_DISPLAY
	uint8_t i = sizeof(display_value) * CHAR_BIT - 1;
	asm ( //double-dabble, no jmp loops to save cycles (I'm sure C compiler will turn this algorithm into a nightmare for an 8-bit AVR ==> ASM)
		"ldi r16, 0x03 \n\t"	//Load magic value
		"mov __tmp_reg__, r16 \n\t"	//Free r16 to reuse it and minimize clobbering
"L_%=:"	"rol %A0 \n\t"	//Shift everything with carrying
		"rol %B0 \n\t"
		"rol %C0 \n\t"
		"rol %D0 \n\t"
		"mov r16, %C0 \n\t"	//Use r16 to compare nibbles to 0x05 threshold
		"andi r16, 0x0F \n\t"
		"cpi r16, 0x05 \n\t"
		"brlo .+2 \n\t"	//If lower, skip
		"add %C2, __tmp_reg__ \n\t" //Adding the magic value to the nibble
		"swap %C0 \n\t"	//Now process high nibble by swapping
		"mov r16, %C0 \n\t"
		"andi r16, 0x0F \n\t"
		"cpi r16, 0x05 \n\t"
		"brlo .+2 \n\t"
		"add %C2, __tmp_reg__ \n\t"
		"swap %C0 \n\t" //Swap back
		"mov r16, %D0 \n\t" //Repeat for high byte
		"andi r16, 0x0F \n\t"
		"cpi r16, 0x05 \n\t"
		"brlo .+2 \n\t"
		"add %D2, __tmp_reg__ \n\t"
		"swap %D0 \n\t"
		"mov r16, %D0 \n\t"
		"andi r16, 0x0F \n\t"
		"cpi r16, 0x05 \n\t"
		"brlo .+2 \n\t"
		"add %D2, __tmp_reg__ \n\t"
		"swap %D0 \n\t"
		"dec %1 \n\t" //Decrement bit counter
		"brne L_%= \n\t" //If non-zero, loop
		"rol %B0 \n\t"	//Last shift with carrying (don't add 0x03s after the final shift!), first nibble is out of the game by this point
		"rol %C0 \n\t"
		"rol %D0 \n\t"
		: "=d" (temp), "=r" (i)
		: "0" (temp), "1" (i)
		: "r16"
	);
	temp >>= 16_ui8; //Compiles into MOVW
	for (; i < DISPLAY_DIGITS; ++i)
	{
		uint8_t t = 0x0F & temp;
		asm ("swap %0" : "=r" (t) : "0" (t)); //Saves 3 instructions compared to C-shift
		display_value_nibbles[i] = t;
		temp >>= 4_ui8;
	}
#endif
#ifdef DEBUG_CURRENT
{
	char buf_dbg[8];
	uart_puts(itoa(current_interval, buf_dbg, 16));
	uart_putc('\n');
	//uart_puts(itoa(a, buf_dbg, 16));
	//uart_putc('\n');
}
#endif
#ifdef DEBUG_VERBOSE
	uart_puts("Finished\n");
#endif
}

void process_enabling(void)
{
	if (enable)
	{
		if (PINB & BV8(PIN_ENA))
		{
			//OK
			PORTB |= BV8(PIN_LED);
		}
		else
		{
			//Start the motor up
			#ifdef DEBUG
				uart_puts_P("Motor has been enabled\n");
			#endif // DEBUG
			enable_pwm();
			PORTB ^= BV8(PIN_ENA);
		}
	}
	else
	{
		if (PINB & BV8(PIN_ENA))
		{
			//Stop the motor
			if (current_interval >= TIMER_MAX)
			{
				PORTB ^= BV8(PIN_ENA);
				disable_pwm();
				reset_current();
				#ifdef DEBUG
					uart_puts_P("Motor has been disabled\n");
				#endif
			}
		}
		else
		{
			//OK
			PORTB &= ~BV8(PIN_LED);
		}
	}
}

void enable_pwm(void)
{
	// OCR1B is used as compare match, fixed to 2.5uS.
	OCR1BH = 0;
	OCR1BL = PULSE_LENGTH; //Counting from 0
}
void disable_pwm(void)
{
	// OCR1B is used as compare match
	OCR1BH = 0;
	OCR1BL = 0; //Counting from 0, initialize to BOTTOM so that the PWM is off
}
void drive::set_microstepping(uint16_t _microstepping)
{
	//Phase & frequency correct PWM operates in dual-slope mode - divide by 2
	//Multiply by 10 to get an extra resolution digit (fixed-point arithmetic), and by 60 to convert to min^-1
	microstepping_precalculated = (F_CPU / _microstepping) * (60u * 10u
#if USE_DUAL_SLOPE
	/ 2u
#endif
	);
	microstepping = _microstepping;
}
void drive::set_speed(float _speed)
{
	set_interval(static_cast<interval_t>(microstepping_precalculated / (_speed * 10u)));
}
void drive::set_interval(interval_t _interval)
{
	target_interval = _interval;
#ifdef DEBUG_CURRENT
	char dbg_buf[16];
	uart_puts("T:");
	uart_puts(itoa(target_interval, dbg_buf, 10));
	uart_putc('\n');
#endif // DEBUG_CURRENT
}
void drive::set_direction(bool ccw)
{
	PORTB &= ~BV8(PIN_DIR);
	if (ccw) PORTB |= BV8(PIN_DIR);
}
void drive::set_target(float _rotations)
{
	set_direction(_rotations < 0);
	reset_current();
	uint32_t dec;
	if (target_interval < PRECISE_DECELERATION_LIMIT)
	{	//Precise
		if (target_interval < arraySize(precise_deceleration_table_high))
		{	//High part
			static_assert(sizeof(precise_high_t) == sizeof(uint32_t), "Table types not in sync!");
			dec = pgm_read_dword(&(precise_deceleration_table_high[target_interval]));
		}
		else
		{  //Low part
			static_assert(sizeof(precise_low_t) == sizeof(uint16_t), "Table types not in sync!");
			dec = pgm_read_word(&(precise_deceleration_table_low[target_interval - arraySize(precise_deceleration_table_high)]));
		}
	}
	else
	{	//Generic
		if (target_interval < arraySize(deceleration_table_high))
		{	//High part
			static_assert(sizeof(decel_high_t) == sizeof(uint16_t), "Table types not in sync!");
			dec = pgm_read_word(&(deceleration_table_high[target_interval]));
		}
		else
		{	//Low part
			static_assert(sizeof(decel_low_t) == sizeof(uint8_t), "Table types not in sync!");
			dec = pgm_read_byte(&(deceleration_table_low[target_interval - arraySize(precise_deceleration_table_high)]));
		}
	}
#ifdef DEBUG
	char dbg_buf[8];
	uart_puts("D=");
	uart_puts(itoa(dec, dbg_buf, 16));
	uart_puts("h\n");
#endif // DEBUG
	target_step_limit = static_cast<uint32_t>(fabs(_rotations) * microstepping);
	if (dec > target_step_limit) target_step_limit = 0;
	else target_step_limit -= dec;
}
void drive::run(float _rotations)
{
	drive::set_target(_rotations);
	enable = true;
}
void drive::run()
{
	reset_current();
	enable = true;
}
void drive::stop()
{
	enable = false;
}
void drive::reset_current()
{
	drive_steps_low = 0;
	drive_steps_high = 0;
	current_steps = 0;
}
uint32_t drive::get_target_steps()
{
	return target_step_limit;
}
uint32_t drive::get_current_steps()
{
	return current_steps;
}
interval_t drive::get_current_interval()
{
	return current_interval;
}
interval_t drive::get_target_interval()
{
	return target_interval;
}
bool drive::get_alarm()
{
	return PINC & BV8(PIN_ALM);
}
interval_t drive::get_current_acceleration()
{
	return current_acceleration;
}

#pragma endregion Functions