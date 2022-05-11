#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>
#include <inttypes.h>
#include <util/delay.h>

#include "steering.h"

volatile uint16_t angle_offset = 1610;

void PWM_init() {
	// Set fast PWM mode with non-inverted output for engine
	TCCR0A = (1<<WGM00) | (1<<WGM01) |  (1<<COM0A1);
	TCCR0B = (0<<WGM02) | (0<<CS02) | (1<<CS01) | (0<<CS00);
	
	// Set fast PWM mode with non-inverted output for steering
	TCCR1A = (1<<WGM11) | (0<<WGM10) | (1<<COM1A1);
	TCCR1B = (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
	
	DDRB |= 0x08; // Output port engine (Yellow wire)
	DDRD |= (1<<5); // Output port steering (Green wire)
	DDRD |= (1<<3); // Output port break
	PORTD = (0<<3);
	
	sei();        // Enable interrupt
	
	ICR1 = 20000; // Servo frequency
	OCR0A = 0;    // Initial speed
	OCR1A = angle_offset; // Initial steering output
}


void safety_timer_init() {
	TCCR3A = 0;
	TCCR3B = (1<<WGM32) | (1<<CS32);  // Set CTC and prescaler = 256
	TIMSK3 = (1<<OCIE3A); // Set OCR1A as top-value
	OCR3A = 31250;  // Gives interrupt each second
}


ISR (TIMER3_COMPA_vect) {
	set_speed(0);  // Stops the car if it receives no signals
}


/* Ensures the speed stays within reasonable bounds (100 < OCR0A < 0, max OCR0A = 255) */
int16_t set_speed(int16_t speed) {
	if (speed > MAX_THROTTLE) {
		speed = MAX_THROTTLE;
	} else if (speed < 0) {
		speed = 0;
	}
	if (speed == 0) {
		OCR0A = 0;
		PORTD = (1<<3);
	} else {
		PORTD = (0<<3);
		OCR0A = speed;
	}
	return speed;    // Return actual throttle value set
}


/* Ensures the steering angle stays within bounds (1230 < OCR1A < 1950). Neutral when OCR1A = angle_offset */
int16_t set_steering(int16_t steering) {  // uint16_t didn't work.
	int16_t steering_pwm = angle_offset - steering;
	if (steering_pwm > 1950) {
		steering_pwm = 1950;    // Max turn left
	} else if (steering_pwm < 1230) {
		steering_pwm = 1230;    // Max turn right
	}
	OCR1A = steering_pwm;
	return angle_offset - steering_pwm;    // Return actual steering value set
}


int16_t calculate_speed(int16_t cur_vel, int16_t ref_vel, int16_t speed_KP, int16_t int_sum) {
	int16_t y = (speed_KP*(ref_vel - cur_vel))/10 + int_sum;
	return y;
}


/*  steering control
	ref_lat: lateral distance to reference point from the closest wall
	cur_lat: the cars current lateral distance from the closest wall
	cur_vel: the cars current speed
	cur_ang: the cars current angle relative to the road
*/
int16_t calculate_steering(int16_t cur_vel, int16_t cur_lat, int16_t cur_ang, int16_t steering_KP, int16_t steering_KD) {
	int16_t y = (steering_KP*(cur_lat-20))/10 + (steering_KD*cur_ang)/10;
	return y;
}

int16_t calculate_steering_turning(int16_t cur_vel, int16_t cur_lat, int16_t cur_ang, int16_t turn_KP, int16_t turn_KD) {
	int16_t y = (turn_KD*cur_ang)/10;
	return y;
}
