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
	int16_t cur_vel = 0;
	int16_t ref_vel = 0;
	int16_t cur_lat = 0;
	int16_t ref_lat = 0;
	int16_t cur_ang = 0;
	int16_t steering_KP = 0;
	int16_t steering_KD = 0;
	int16_t speed_KP = 0;
	
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
			bool cur_ang_bool = false;    // Current angle
			bool steering_KP_bool = false;// KP parameter for steering
			bool steering_KD_bool = false;// KD parameter for steering
			bool speed_KP_bool = false;   // KP parameter for speed
			
			for (int i=0; i<len; ++i) {
				switch (message_names[i]) {
					
					case STEERING_MANUAL_GAS:
						man_gas_bool = true;
						break;		
					case STEERING_MANUAL_ANG:
						man_steer_bool = true;
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
					case STEERING_CUR_ANG:
						cur_ang_bool = true;
						cur_ang = messages[i];
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
						
					default:
						break;
				}
			}
			
			if (man_gas_bool && man_steer_bool) {
				// Manual mode
				for (int i=0; i<len; ++i) {
					switch (message_names[i]) {
						
						case STEERING_MANUAL_GAS:
							set_speed(messages[i]);
							break;
						case STEERING_MANUAL_ANG:
							set_steering(restore_signed(messages[i]));
							break;
					}
				}
				reset_safety_timer();
				
			} else if (cur_vel_bool && ref_vel_bool && cur_lat_bool && ref_lat_bool && cur_ang_bool && steering_KP_bool && steering_KD_bool && speed_KP_bool) {
				// Automatic modes
				reset_safety_timer();
				int16_t y = calculate_steering(cur_vel, cur_lat, ref_lat, cur_ang, steering_KP, steering_KD);
				git sset_steering(y);
				
			} else {
				// TODO
			}
		}
	}
}
