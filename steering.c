#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <inttypes.h>

#include "steering.h"


void PWM_init() {
	// Set fast PWM mode with non-inverted output for engine
	TCCR0A = (1<<WGM00) | (1<<WGM01) |  (1<<COM0A1);
	TCCR0B = (0<<WGM02) | (0<<CS02) | (1<<CS01) | (0<<CS00);
	
	// Set fast PWM mode with non-inverted output for steering
	TCCR1A = (1<<WGM11) | (0<<WGM10) | (1<<COM1A1);
	TCCR1B = (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
	
	DDRB |= 0x08; // Output port engine (Yellow wire)
	DDRD |= (1<<5); // Output port steering (Green wire)
	
	sei();        // Enable interrupt
	
	ICR1 = 20000; // Servo frequency
	OCR0A = 0;    // Initial speed
	OCR1A = 1590; // Initial steering output
}


/* Ensures the speed stays within reasonable bounds (100 < OCR0A < 0, max OCR0A = 255) */
void set_speed(int16_t speed) {
	if (speed > 100) {
		OCR0A = 100;
	} else if (speed < 0) {
		OCR0A = 0;
	} else {
		OCR0A = speed;
	}
}


/* Ensures the steering angle stays within bounds (1230 < OCR1A < 1950). Neutral when OCR1A = 1590 */
void set_steering(int16_t steering) {  // uint16_t didn't work.
	if (1590 + steering < 1230) {
		OCR1A = 1230;
	} else if (1590 + steering > 1950) {
		OCR1A = 1950;
	} else {
		OCR1A = 1590 + steering;
		
	}
}


/*  steering control
	ref_lat: lateral distance to reference point from the closest wall
	cur_lat: the cars current lateral distance from the closest wall
	cur_vel: the cars current speed
	cur_ang: the cars current angle relative to the road
*/
void set_steering_pwm(int16_t cur_vel, int16_t cur_lat, int16_t ref_lat, int16_t cur_ang) {
	uint16_t KP = 1;  // Proportional control variable
	uint16_t KD = 1;  // Derivative control variable
	uint16_t y = KP*(ref_lat - cur_lat) + KD*(cur_vel*sin(cur_ang));
	set_steering(y);
}