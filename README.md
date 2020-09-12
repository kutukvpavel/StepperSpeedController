# StepperSpeedController
ATmega328-based step/dir controller capable of gradual acceleration/deceleration with 200kHz frequency limit.

This is an Atmel Studio 7.0 project, initially developed to overcome pulse frequency limitations of well-known AccelStepper. It's primary use was testing of closed-loop servo drives (e.g. Leadshine ones) with Step/Dir interface and it was put together in a couple of evenings. Out-of-the-box it expects a potentiometer to control speed, a RUN/STOP button and a UART connection to control travel distance. Distance control was added to the project later than speed control, therefore it's still a work in progress, and currently extra steps required for deceleration are calculated inaccurately.

To achieve high pulse frequency three things are used:
  - hardware-based pulse generation (16-bit Timer1 is used)
  - lookup-table-based acceleration and input resolution mapping
  - a lot of inline ASM and NAKED interrupts
  
Additional features:
  - ADC noise filtering (moving average)
  - BCD display ready (6 upper bits of PORTD are reserved for multiplexed BCD output: 4 upper bits are the BCD digit code and PORTD2-3 are the digit index in binary, update frequency about 1kHz), intended to be used with a 4-digit 7-segment display and external BCD-to-7-seg decoder plus something like 74 series multiplexer
  - saving some parameters to EEPROM
  
BCD was implemented mainly because it's cheap and simple. Now I2C shared bus for all keypads/displays seems like a good choice.

Currently, there's a bunch of known limitations:
  - acceleration profile can be edited only by recompiling the source, since lookup tables are used (calculations are performed in Wolfram Mathematica 10 for now)
  - speed resolution at high frequencies is not perfect, partially due to Phase & Frequency Correct PWM timer mode being used (considering to try Fast PWM, there are caveats)
  - lookup-table approach requires a table of precalculated times of deceleration, and current mathematical model seems to miss something
  - no standalone (not UART-based) UI for distance control exists, because we're running out of IO pins (hardware I2C is an option, but I don't have the hardware on hand yet)

This was a fun project aimed to push low-end-ish AVR8 to its limits and learn AVR (inline-) assembler, but the more it approaches "a high-speed AccelSteper alternative with some compromises", the more tedious the development of it as a library gets. So, I think the right way to go is to just switch to STM32 for the sake of development time.

P.S. Compilation requires at least GCC 5.x.x with -std=gnu99 (for C) and -std=gnu++11 (for C++).
