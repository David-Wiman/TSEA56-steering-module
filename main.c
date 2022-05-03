#include <avr/io.h>
#include <math.h>
#include <inttypes.h>
#include <avr/interrupt.h>

#include "steering.h"
#include "i2c/avr_i2c.h"
#include "common/i2c_common.h"
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
	int16_t cur_ang = 0;
	int16_t steering_KP = 0;
	int16_t steering_KD = 0;
	int16_t speed_KP = 2;
	int16_t speed_KI = 2;
	int16_t turn_KP = 0;
	int16_t turn_KD = 0;
	int16_t regulation_mode = -1;
	
	volatile int16_t throttle_set = 0;
	volatile int16_t steering_set = 0;
	volatile int16_t old_throttle_set = 0;
	volatile int16_t old_steering_set = 0;
	
	volatile int16_t  DUMMY_vel = 0;
	
	int16_t y = 0;
	int16_t speed_I_sum = 0;  // Integration sum for the speed regulator
	
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
			bool cur_ang_bool = false;    // Current angle
			bool steering_KP_bool = false;// KP parameter for steering
			bool steering_KD_bool = false;// KD parameter for steering
			bool speed_KP_bool = false;   // KP parameter for speed
			bool speed_KI_bool = false;   // KI parameter for speed
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
						man_ang = restore_signed(messages[i]);
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
						cur_lat = restore_signed(messages[i]);
						break;
					case STEERING_CUR_ANG:
						cur_ang_bool = true;
						cur_ang = restore_signed(messages[i]);
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
					case STEERING_SPEED_KI:
						speed_KI_bool = true;
						speed_KI = messages[i];
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
			
			switch (regulation_mode) {
				
				case REGULATION_MODE_MANUAL: ;  // Manual mode 
					throttle_set = set_speed(man_gas);
					steering_set = set_steering(man_ang);
					
					if ((throttle_set != old_throttle_set) || (steering_set != old_steering_set)) {
						uint16_t message_names_send[] = {STEERING_RETURN_GAS, STEERING_RETURN_ANG};
						uint16_t messages_send[] = {package_signed(throttle_set), package_signed(steering_set)};
						I2C_pack(message_names_send, messages_send, 2);
						old_throttle_set = throttle_set;
						old_steering_set = steering_set;
					}
					
					reset_safety_timer();
					break;
				
				case REGULATION_MODE_AUTO_FORWARD:  // Autonomous forward
					if (ref_vel == 0) {
						speed_I_sum = 0;
					}
					
					speed_I_sum = speed_I_sum + (speed_KI*(ref_vel - cur_vel))/100;
					
					/*if (speed_I_sum > max_throttle) {
						speed_I_sum = max_throttle;
					} else if (speed_I_sum < 0) {
						speed_I_sum = 0;
					}*/
					
					y = calculate_speed(cur_vel, ref_vel, speed_KP, speed_I_sum);
					DUMMY_vel = y;
					
					// To kick start the car
					if ((cur_vel == 0) && (y > 0)) {
						y = 150;
						speed_I_sum = 100;
					}
					
					set_speed(y);
					
					int16_t steering = calculate_steering_turning(cur_vel, cur_lat, cur_ang, turn_KP, turn_KD);
					set_steering(steering);
					
					reset_safety_timer();
					break;
				
				case REGULATION_MODE_AUTO_TURN:
					speed_I_sum = speed_I_sum + (speed_KI*(ref_vel - cur_vel))/100;
					
					y = calculate_speed(cur_vel, ref_vel, speed_KP, speed_I_sum);
					DUMMY_vel = y;
					
					// To kick start the car
					if ((cur_vel == 0) && (y > 0)) {
						y = 150;
						speed_I_sum = 100;
					}
					
					set_speed(y);
					
					int16_t steering_turn = calculate_steering_turning(cur_vel, cur_lat, cur_ang, turn_KP, turn_KD);
					set_steering(steering_turn);
					
					reset_safety_timer();
					break;
					
				default:
					break;
			}
		}
	}
}