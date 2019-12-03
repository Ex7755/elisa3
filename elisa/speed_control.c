
#include "speed_control.h"


void init_speed_control() {
	//p_speed_control = 40;
	//d_speed_control = 5;
	//i_speed_control = 10;
	//i_limit_speed_control = 3200;
	//k_ff_speed_control_left = INIT_KFF;
	//k_ff_speed_control_right = INIT_KFF;
}

void start_speed_control_right(signed int *pwm_right) {

	// the input paramter is the current desired speed, expressed in the pwm range (-512..512).

	if(*pwm_right==0) {
		delta_right_speed_sum = 0;		// reset the sum of the error for the I parameter
		delta_right_speed_current = 0;
		delta_right_speed_prev = 0;
		return;
	}

	// compute the current error between the desired and measured speed
	delta_right_speed_prev = delta_right_speed_current;
	if(*pwm_right >= 0) {
		delta_right_speed_current = (*pwm_right) - last_right_vel;
	} else {
		delta_right_speed_current = (*pwm_right) + last_right_vel;
	}

	// sum the errors
	delta_right_speed_sum += delta_right_speed_current;

	if(delta_right_speed_sum > I_LIMIT) {
		delta_right_speed_sum = I_LIMIT;
	}else if(delta_right_speed_sum < -I_LIMIT) {
		delta_right_speed_sum = -I_LIMIT;
	}		

	// pwm out = feed forward * desired speed + P * current error - D * (current error - previous error) + I * error sum
	// in this case feed forward = 8
	pwm_right_speed_controller = (signed long int)((*pwm_right) << 3);
	pwm_right_speed_controller += (signed long int)(delta_right_speed_current*P_PART);
	pwm_right_speed_controller += (signed long int)((delta_right_speed_current-delta_right_speed_prev)*D_PART);
	pwm_right_speed_controller += (signed long int)(delta_right_speed_sum*I_PART);

	// avoid changing motion direction
	if(pwm_right_speed_controller < 0 && *pwm_right >= 0) {	
		pwm_right_speed_controller = 0;
	}
	if(pwm_right_speed_controller > 0 && *pwm_right < 0 ) {
		pwm_right_speed_controller = 0;
	}

	if (pwm_right_speed_controller>MAX_PWM) pwm_right_speed_controller=MAX_PWM;
	if (pwm_right_speed_controller<-MAX_PWM) pwm_right_speed_controller=-MAX_PWM;

	// since the pwm_left_speed_controller goes from -24000 to 24000 then the pwm_left 
	// has to be scaled to remain in the range -512..512
	*pwm_right = (signed int)(pwm_right_speed_controller>>4);

	// avoid stopping the motors if desired speed is different from zero
	if(pwm_right_desired_to_control > 0) {
		*pwm_right += 1;
	} else {
		*pwm_right -= 1;
	}

	// the feed forward is composed by the previous scale factor (x8) and by an offset
	/*
	if(*pwm_right > 0) {
		*pwm_right += 30;
	} else if(*pwm_right < 0) {
		*pwm_right -= 30;
	}
	*/

	if (*pwm_right>(MAX_MOTORS_PWM/2)) *pwm_right=(MAX_MOTORS_PWM/2);
    if (*pwm_right<-(MAX_MOTORS_PWM/2)) *pwm_right=-(MAX_MOTORS_PWM/2);

}

void start_speed_control_left(signed int *pwm_left) {

	// the input paramter is the current desired speed, expressed in the pwm range (-512..512).

	if(*pwm_left==0) {
		delta_left_speed_sum = 0;		// reset the sum of the error for the I parameter
		delta_left_speed_current = 0;
		delta_left_speed_prev = 0;
		return;
	}

	// compute the current error between the desired and measured speed
	delta_left_speed_prev = delta_left_speed_current; 
	if(*pwm_left >= 0) {
		delta_left_speed_current = (*pwm_left) - last_left_vel; 
	} else {
		delta_left_speed_current = (*pwm_left) + last_left_vel; 
	}
	// sum the errors
	delta_left_speed_sum += delta_left_speed_current;

	if(delta_left_speed_sum > I_LIMIT) {
		delta_left_speed_sum = I_LIMIT;
	} else if(delta_left_speed_sum < -I_LIMIT) {
		delta_left_speed_sum = -I_LIMIT;
	}
	    
	// pwm out = feed forward * desired speed + P * current error - D * (current error - previous error) + I * error sum
	// in this case feed forward = 8
	pwm_left_speed_controller = (signed long int)((*pwm_left) << 3);
	pwm_left_speed_controller += (signed long int)(delta_left_speed_current*P_PART);
	pwm_left_speed_controller += (signed long int)((delta_left_speed_current-delta_left_speed_prev)*D_PART);
	pwm_left_speed_controller += (signed long int)(delta_left_speed_sum*I_PART);

	// avoid changing motion direction
	if(pwm_left_speed_controller < 0 && *pwm_left >= 0) {
		pwm_left_speed_controller = 0;
	}
	if(pwm_left_speed_controller > 0 && *pwm_left < 0 ) {
		pwm_left_speed_controller = 0;
	}

	if (pwm_left_speed_controller>MAX_PWM) pwm_left_speed_controller=MAX_PWM;
	if (pwm_left_speed_controller<-MAX_PWM) pwm_left_speed_controller=-MAX_PWM;

	// since the pwm_left_speed_controller goes from -24000 to 24000 then the pwm_left 
	// has to be scaled to remain in the range -512..512
	*pwm_left = (signed int)(pwm_left_speed_controller>>4);

	// avoid stopping the motors if desired speed is different from zero
	if(pwm_left_desired_to_control > 0) {
		*pwm_left += 1;
	} else {
		*pwm_left -= 1;
	}

	// the feed forward is composed by the previous scale factor (x8) and by an offset
	/*
	if(*pwm_left > 0) {
		*pwm_left += 30;
	} else if(*pwm_left < 0) {
		*pwm_left -= 30;
	}
	*/

	if (*pwm_left>(MAX_MOTORS_PWM/2)) *pwm_left=(MAX_MOTORS_PWM/2);
    if (*pwm_left<-(MAX_MOTORS_PWM/2)) *pwm_left=-(MAX_MOTORS_PWM/2);

}

