#include"PWM.h"

// Servo times - to be entered as timer1 ticks (using US2TIMER1).
// This must be updated with interrupts disabled.
volatile static uint16_t servoTime[N_SERVOS];

// Servo output allocation (on a single port currently).
const static uint8_t servoOutMask[N_SERVOS] = {
    0b00000001, // PX0
    0b00000010, // PX1
    0b00000100, // PX2
    //0b00001000, // PX3
    //0b00010000, // PX4
    //0b00100000, // PX5
    //0b01000000, // PX6
    //0b10000000, // PX7
};

void servoStart(void)
{
    // Outputs
    SERVO_DDR |= SERVO_MASK;
    // Setupt a first compare match
    OCR1A = TCNT1 + US2TIMER1(100);
    // start timer 1 with no prescaler
    //TCCR1B |= (1 << CS10);      
    // Enable interrupt
    TIMSK |= (1 << OCIE1A);
}

void servoSet(uint8_t servo, uint16_t time /* microseconds */)
{
    uint16_t ticks = US2TIMER1(time);
    cli();
    servoTime[servo] = ticks;
    sei();
}

void servoResetPos(void){
    uint8_t i;
    for(i = 0; i < N_SERVOS; i++) {
        servoTime[i] = US2TIMER1(SERVO_MID);
    }
}
#if 0
int main(void)
{
    // Some test times
    uint8_t i;
    for(i = 0; i < N_SERVOS; i++) {
        servoTime[i] = US2TIMER1(SERVO_MID);
    }
#if N_SERVOS > 2   
    servoTime[2] = US2TIMER1(SERVO_MIN);
#endif
    servoTime[N_SERVOS-1] = US2TIMER1(SERVO_MAX);
   
    servoStart();
    sei();
   
    while(1) {
       
	 uint16_t time;
	uint8_t w;
       	
	servoSet(0, SERVO_MID-300);
	servoSet(1, SERVO_MID-300);	
	servoSet(2, SERVO_MID-300);		
	_delay_ms(1000);
	servoSet(0, SERVO_MID);
	servoSet(1, SERVO_MID);
	servoSet(2, SERVO_MID);		
	_delay_ms(1000);
	servoSet(0, SERVO_MID+300);
	servoSet(1, SERVO_MID+300);
	servoSet(2, SERVO_MID+300);		
	_delay_ms(1000);
	
        // Test update of servo 0
       /* for(time = 750; time <= 2750; time += 1) {
            servoSet(0, time);
             _delay_ms(1);
        }
	for(;time >= 750; time -= 1) {
            servoSet(0, time);
		_delay_ms(1);
        }*/
    }
}
#endif

ISR(TIMER1_COMPA_vect)
{
    static uint16_t nextStart;
    static uint8_t servo;
    static bool outputHigh = true;
    uint16_t currentTime = OCR1A;
    uint8_t mask = servoOutMask[servo];
   
    if (outputHigh) {
        SERVO_PORT |= mask;
        // Set the end time for the servo pulse
        OCR1A = currentTime + servoTime[servo];
        nextStart = currentTime + US2TIMER1(SERVO_TIME_DIV);
    } else {
        SERVO_PORT &= ~mask;
        if (++servo == N_SERVOS) {
            servo = 0;
        }
        OCR1A = nextStart;
    }
    outputHigh = !outputHigh;
} 
