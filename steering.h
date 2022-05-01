#ifndef STEERING_H
#define STEERING_H

#include <inttypes.h>

static uint8_t max_throttle = 150;

void PWM_init();

void safety_timer_init();

inline void reset_safety_timer() {
	TCNT3 = 0;
}

int16_t set_speed(int16_t speed);

int16_t set_steering(int16_t steering);

int16_t calculate_speed(int16_t cur_vel, int16_t ref_vel, int16_t speed_KP, int16_t int_sum);

int16_t calculate_steering(int16_t cur_vel, int16_t cur_lat, int16_t cur_ang, int16_t steering_KP, int16_t steering_KD);

int16_t calculate_steering_turning(int16_t cur_vel, int16_t cur_lat, int16_t cur_ang, int16_t steering_KP, int16_t steering_KD);

#endif // STEERING_H