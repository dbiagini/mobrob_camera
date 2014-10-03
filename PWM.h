#ifndef F_CPU
  #define F_CPU 8000000L
#endif
#include <stdint.h>
#include <stdbool.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


#define SERVO_PORT  PORTA
#define SERVO_DDR   DDRA

// Upto 8 servos (since pulses are generated in
// sequence + only one port is used).
#define N_SERVOS    3

// Servo times (this is Futaba timing).
#define SERVO_MIN    700 // microseconds
#define SERVO_MAX   2735 // microseconds
#define SERVO_MID   (SERVO_MIN + SERVO_MAX) / 2

// Time between servo pulses. 
#define SERVO_FRAME 20000 // microseconds (50Hz)

// Time slot available for each servo.
#define SERVO_TIME_DIV (SERVO_FRAME / N_SERVOS)

#if (SERVO_TIME_DIV < SERVO_MAX + 50)
#warning "Output fewer servo signals or increase SERVO_FRAME"
#endif
#if ((SERVO_TIME_DIV * (F_CPU / 1000000UL)) >= 0xFF00)
#warning "Output more servo signals or decrease SERVO_FRAME (or use the prescaler)"
#endif

// Computing timer ticks given microseconds.
// Note, this version works out ok with even MHz F_CPU (e.g., 1, 2, 4, 8, 16 MHz).
// (Not a good idea to have this end up as a floating point operation)
#define US2TIMER1(us) ((us) * (uint16_t)(F_CPU / 1E6))

// Servo mask is just the masks ored.
#define SERVO_MASK 0xff

void servoStart(void);

void servoSet(uint8_t servo, uint16_t time /* microseconds */);

void servoResetPos(void);
