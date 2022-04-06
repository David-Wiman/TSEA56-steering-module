#include <avr/io.h>
#include <math.h>
#include <inttypes.h>

#include "steering.h"
#include "../communication-module/common/avr_i2c.h"
#include "../communication-module/common/i2c_common.h"


int main() {
	PWM_init();
	I2C_init(0x51);
	
	uint16_t message_names[16];
	uint16_t messages[16];
	// Values to be extracted from messages
	int16_t cur_vel = 0;
	int16_t ref_vel = 0;
	int16_t cur_lat = 0;
	int16_t ref_lat = 0;
	int16_t cur_ang = 0;

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
						break;
					case STEERING_REF_VEL:
						ref_vel_bool = true;
						break;
					case STEERING_CUR_LAT:
						cur_lat_bool = true;
						break;
					case STEERING_REF_LAT:
						ref_lat_bool = true;
						break;
					case STEERING_CUR_ANG:
						cur_ang_bool = true;
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
			} else if (cur_vel_bool && ref_vel_bool && cur_lat_bool && ref_lat_bool && cur_ang_bool) {
				// Automatic modes
				for (int i=0; i<len; ++i) {
					switch (message_names[i]) {
						// Set values
						case STEERING_CUR_VEL:
							cur_vel = messages[i];
							break;
						case STEERING_REF_VEL:
							ref_vel = messages[i];
							break;
						case STEERING_CUR_LAT:
							cur_lat = messages[i];
							break;
						case STEERING_REF_LAT:
							ref_lat = messages[i];
							break;
						case STEERING_CUR_ANG:
							cur_ang = messages[i];
							break;
					}
				}

				set_steering_pwm(cur_vel, cur_lat, ref_lat, cur_ang);
			} else {
				// TODO
			}
		}
	}
}
