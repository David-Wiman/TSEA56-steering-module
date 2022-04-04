#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

void PWM_init() {
	// Set fast PWM mode with non-inverted output for engine
	TCCR0A = (1<<WGM00) | (1<<WGM01) |  (1<<COM0A1);
	TCCR0B = (0<<WGM02) | (0<<CS02) | (1<<CS01) | (0<<CS00);
	
	// Set fast PWM mode with non-inverted output for steering
	TCCR1A = (1<<WGM11) | (0<<WGM10) | (1<<COM1A1);
	TCCR1B = (1<<WGM13) | (1<<WGM12) | (0<<CS12) | (1<<CS11) | (0<<CS10);
	
	DDRB |= 0x08; // Output port engine (Yellow wire)
	DDRD |= (1<<5); // Output port steering (Green wire)
	ICR1 = 20000; // Servo frequency
	sei();        // Enable interrupt
}


// Limit the speed to max 100 (of 255)
int limit_speed(int speed) {
	if (speed > 100) {
		return 100;
	} else {
		return speed;
	}
}



/*  steering control
	r_lat: lateral distance to reference point from the closest wall
	l_lat: the cars current lateral distance from the closest wall
	cur_speed: the cars current speed
	cur_angle: the cars current angle relative to the road
*/
int set_steering_pwm(int r_lat, int l_lat, int cur_speed, int cur_angle) {
	uint8_t KP = 1;  // Proportional control variable
	uint8_t KD = 1;  // Derivative control variable
	uint16_t y = KP*(r_lat - l_lat) + KD*(cur_speed*sin(cur_angle));
	if (y < -360) {
		return -360;
	} else if (y > 360) {
		return 360;
	} else {
		return y;
	}
}


/*  Set speed and angle values
	The car is in neutral mode when OCR1A = 1590
	Maximal left-turn the car is capable of occurs at OCR1A = 1590 + (-360)
	Maximal right-turn the car is capable of occurs at OCR1A = 1590 + 360
	The above is regulated by set_steering_pwm
*/

void PWM_SET(int steering_angle, int throttle) {
	OCR0A = limit_speed(throttle);					// Engine duty cycle
	OCR1A = 1590 + set_steering_pwm(100, 80, steering_angle, 0);			// Steering duty cycle
}


int main ()
{
	PWM_init();

	while (1)
	{	
		PWM_SET(-360, 0);
	}
}
