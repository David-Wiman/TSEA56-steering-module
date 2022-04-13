#include <avr/io.h>
#include <math.h>
#include <inttypes.h>
#include <avr/interrupt.h>

#include "steering.h"
#include "../communication-module/common/avr_i2c.h"
#include "../communication-module/common/i2c_common.h"
#include <util/delay.h>


int main() {
	PWM_init();
	safety_timer_init();
	I2C_init(0x51);
	
	uint16_t message_names[16];
	uint16_t messages[16];
	// Values to be extracted from messages
	int16_t man_gas = 0;
	int16_t man_ang = 0;
	int16_t cur_vel = 0;
	int16_t ref_vel = 0;
	int16_t cur_lat = 0;
	int16_t ref_lat = 0;
	int16_t cur_ang_left = 0;
	int16_t cur_ang_right = 0;
	int16_t cur_ang_avr = 0;
	int16_t steering_KP = 0;
	int16_t steering_KD = 0;
	int16_t speed_KP = 0;
	int16_t turn_KP = 0;
	int16_t turn_KD = 0;
	int16_t regulation_mode = 0;
	
	while (1) {	
		if (i2c_new_data) {
			i2c_new_data = false;
			int len = I2C_unpack(message_names, messages); 
			
			// Bools checking whether we have automatic mode or not
			bool man_gas_bool = false;    // Manual gas
			bool man_steer_bool = false;  // Manual steering
			bool cur_vel_bool = false;    // Current velocity
			bool ref_vel_bool = false;    // Reference velocity
			bool cur_lat_bool = false;    // Current lateral distance
			bool ref_lat_bool = false;    // Reference lateral distance
			bool cur_ang_left_bool = false;    // Current angle
			bool cur_ang_right_bool = false;    // Current angle
			bool steering_KP_bool = false;// KP parameter for steering
			bool steering_KD_bool = false;// KD parameter for steering
			bool speed_KP_bool = false;   // KP parameter for speed
			bool turn_KP_bool = false;   // KP parameter for turning
			bool turn_KD_bool = false;   // KD parameter for turning
			bool regulation_mode_bool = false;   // Regulation mode
			
			for (int i=0; i<len; ++i) {
				switch (message_names[i]) {
					
					case STEERING_MANUAL_GAS:
						man_gas_bool = true;
						man_gas = messages[i];
						break;		
					case STEERING_MANUAL_ANG:
						man_steer_bool = true;
						man_ang = messages[i];
						break;
						
					case STEERING_CUR_VEL:
						cur_vel_bool = true;
						cur_vel = messages[i];
						break;
					case STEERING_REF_VEL:
						ref_vel_bool = true;
						ref_vel = messages[i];
						break;
					case STEERING_CUR_LAT:
						cur_lat_bool = true;
						cur_lat = messages[i];
						break;
					case STEERING_REF_LAT:
						ref_lat_bool = true;
						ref_lat = messages[i];
						break;
					case STEERING_CUR_ANG_LEFT:
						cur_ang_left_bool = true;
						cur_ang_left = messages[i];
						break;
					case STEERING_CUR_ANG_RIGHT:
						cur_ang_right_bool = true;
						cur_ang_right = messages[i];
						break;
					case STEERING_STEERING_KP:
						steering_KP_bool = true;
						steering_KP = messages[i];  // KP value was multiplied by 1000 previously
						break;
					case STEERING_STEERING_KD:
						steering_KD_bool = true;
						steering_KD = messages[i];
						break;
					case STEERING_SPEED_KP:
						speed_KP_bool = true;
						speed_KP = messages[i];
						break;
					case STEERING_TURN_KP:
						turn_KP_bool = true;
						turn_KP = messages[i];
						break;
					case STEERING_TURN_KD:
						turn_KD_bool = true;
						turn_KD = messages[i];
						break;
					case STEERING_REGULATION_MODE:
						regulation_mode_bool = true;
						regulation_mode = messages[i];
						break;
						
					default:
						break;
				}
			}
			
			if (man_gas_bool && man_steer_bool) {
				// Manual mode
				set_speed(man_gas);
				set_steering(man_ang);
				reset_safety_timer();
				
			} else if (cur_vel_bool && ref_vel_bool && cur_lat_bool && ref_lat_bool && cur_ang_left_bool && cur_ang_right_bool && steering_KP_bool && steering_KD_bool && speed_KP_bool && turn_KP_bool && turn_KD_bool && regulation_mode_bool) {
				// Automatic modes
				reset_safety_timer();
				
				if (regulation_mode == 0) { // Drive forward
					
					cur_ang_avr = (cur_ang_left + cur_ang_right)/2;
					int16_t y = calculate_steering(cur_vel, cur_lat, ref_lat, cur_ang_avr, steering_KP, steering_KD);
					set_steering(y);
					
					y = calculate_speed(cur_vel, ref_vel, speed_KP);
					set_speed(y);
				}
				
				else if (regulation_mode == 1) { // Currently turning
					// TODO
				}
			} else {
				// TODO
			}
		}
	}
}
