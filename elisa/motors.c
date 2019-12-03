
#include "motors.h"

void initMotors() {

	// Motor right timer3/pwm
	// Timers clock input = Fosc = 8 MHz
	// Period freq = Fosc/TOP (max timer value) => TOP = Fosc/period freq
	// We need a period time of 10 ms (100 Hz)
	// Using 10-bit resolution (waveform generation mode 7) we have a period of: 8000000/1024 = 7812.5 Hz
	// We need to apply a prescaler to the timer in such a way to get the desired period:
	// 7812.5/100 = 78.125 => ideal prescaler, the nearest one is 1/64 and we get a period of:
	// 8000000/64/1024 = 122 Hz

	TCCR3A = 0;
	TCCR3B = 0;
	TIMSK3 = 0;
	TCCR4A = 0;
	TCCR4B = 0;
	TIMSK4 = 0;

	TCCR3A |= (1 << COM3A1) | (1 << WGM31) | (1 << WGM30); 	// enable OCA; clear on match, set at bottom
	TCCR3A |= (1 << WGM31) | (1 << WGM30);
	TCCR3B |= (1 << WGM32) | (1 << CS31) | (1 << CS30);		// mode 7 => fast-pwm 10 bit; clock prescaler 1/64
	// the values for motors goes from 0 (stopped) to 1023 (max power)
	OCR3A = pwm_right;
	OCR3B = 0;
	TIMSK3 |= (1 << TOIE3);		// Enable timer overflow interrupt

	// stop right motor
	TCCR3A  &= ~(1 << COM3A1) & ~(1 << COM3B1);	// disable OCA and OCB
	PORTE &= ~(1 << 4) & ~(1 << 3);				// output to 0

	// Motor left timer4/pwm
	// same configuration as timer3
	TCCR4A |= (1 << COM4A1) | (1 << WGM41) | (1 << WGM40); 	// enable OCA; clear on match, set at bottom
	TCCR4B |= (1 << WGM42) | (1 << CS41) | (1 << CS40);		// mode 7 => fast-pwm 10 bit; clock prescaler 1/64
	// the values for motors goes from 0 (stopped) to 1024 (max power)
	OCR4A = pwm_left;
	OCR4B = 0;
	TIMSK4 |= (1 << TOIE4);		// Enable timer overflow interrupt
	// stop left motor
	TCCR4A  &= ~(1 << COM4A1) & ~(1 << COM4B1);	// disable OCA and OCB
	PORTH &= ~(1 << 4) & ~(1 << 3);				// output to 0


}

signed int cast_speed(signed int vel) {
    if(vel > MAX_MOTORS_PWM/2) {
        vel = MAX_MOTORS_PWM/2;
    } else if(vel < -(MAX_MOTORS_PWM/2)) {
        vel = -(MAX_MOTORS_PWM/2);
    }
    return vel;
}

void handleMotorsWithNoController() {

	handleSoftAcceleration();

	// compute velocities even if they aren't used
	if(compute_left_vel) {
		last_left_vel = left_vel_sum>>2;
		compute_left_vel = 0;
		left_vel_sum = 0;

		if(pwm_left_desired >= 0) {
			leftMotSteps += (last_left_vel>>3);
		} else {
			leftMotSteps -= (last_left_vel>>3);
		}
	}

	if(compute_right_vel) {
		last_right_vel = right_vel_sum>>2;
		compute_right_vel = 0;
		right_vel_sum = 0;

		if(pwm_right_desired >= 0) {
			rightMotSteps += (last_right_vel>>3);
		} else {
			rightMotSteps -= (last_right_vel>>3);
		}
	}

	pwm_right_working = pwm_intermediate_right_desired*BYTE_TO_MM_S;	// pwm in the range -635..635 (127*BYTE_TO_MM_S)
	pwm_left_working = pwm_intermediate_left_desired*BYTE_TO_MM_S;	
	if(obstacleAvoidanceEnabled) {
		obstacleAvoidance(&pwm_left_working, &pwm_right_working);		// out is in the range -MAX_MOTORS_PWM/2..MAX_MOTORS_PWM/2
	}
	//pwm_left_desired_to_control = cast_speed(pwm_left_working);		// pwm in the range -MAX_MOTORS_PWM/2..MAX_MOTORS_PWM/2
	//pwm_right_desired_to_control = cast_speed(pwm_right_working);

	pwm_left = pwm_left_working;
	pwm_right = pwm_right_working;

	if(pwm_right > 0) {
		OCR3A = (unsigned int)pwm_right;
	} else if(pwm_right < 0) {
		OCR3B = (unsigned int)(-pwm_right);
	} else {
		OCR3A = 0;
		OCR3B = 0;
	}
	if(pwm_left > 0) {
		OCR4A = (unsigned int)pwm_left;
	} else if(pwm_left < 0) {
		OCR4B =(unsigned int)( -pwm_left);
	} else {
		OCR4A = 0;
		OCR4B = 0;
	}

}

void handleMotorsWithSpeedController() {

	handleSoftAcceleration();

	if(calibrateOdomFlag==1) {
		pwm_right_working = pwm_intermediate_right_desired;
		pwm_left_working = pwm_intermediate_left_desired;
	} else {
		if(pwm_intermediate_right_desired >= 0) {		// pwm in the range -127..127
			pwm_right_working = getInputFromSpeed(pwm_intermediate_right_desired, RIGHT_WHEEL_FW_SC);
		} else {
			pwm_right_working = getInputFromSpeed(pwm_intermediate_right_desired, RIGHT_WHEEL_BW_SC);
		}
		if(pwm_intermediate_left_desired >= 0) {		// pwm in the range -127..127
			pwm_left_working = getInputFromSpeed(pwm_intermediate_left_desired, LEFT_WHEEL_FW_SC);
		} else {
			pwm_left_working = getInputFromSpeed(pwm_intermediate_left_desired, LEFT_WHEEL_BW_SC);
		}
	}

	if(obstacleAvoidanceEnabled) {
		obstacleAvoidance(&pwm_left_working, &pwm_right_working);
	}
	pwm_left_desired_to_control = pwm_left_working;
	pwm_right_desired_to_control = pwm_right_working;

	if(compute_left_vel) {

		last_left_vel = left_vel_sum>>2;
		compute_left_vel = 0;
		left_vel_sum = 0;
		
		if(calibrateOdomFlag==1) {
			leftSpeedSumOdom += last_left_vel;
			leftSumCount++;
		}

		getLeftSpeedFromInput();	// get speed in mm/s
		leftDistPrev = leftDist;
		//timeOdometry = getTime100MicroSec()-timeLeftOdom;
		leftDist += ((float)speedLeftFromEnc*((float)(getTime100MicroSec()-timeLeftOdom)*104.0))/1000000.0;	// distance in mm				
		timeLeftOdom = getTime100MicroSec();
		leftMotSteps = (signed long int)leftDist;

/*
		leftMotStepsOld=leftMotSteps;
		if(pwm_left_desired_to_control >= 0) {
			// During field tests we noticed that the measured motors encoders steps (based on measured speed) aren't perfectly proportional
			// with speed, this means that if the robot runs at speed=25 and we get "a" as the encoder value then if the robot runs at
			// speed=50 the related encoder value isn't "2*a".
			// Thus we need to apply a factor to the resulting encoders values based on desired speed: 
			// before => enc = enc + meas_speed*k, where k = 1/8
			// now => enc = enc + meas_speed*k, where k = function(desired_speed)
			// From the tests we extracted the "k" calibration value for a certain robot to be:
			// left => k = 80.8 - 0.384 * desired_speed
			// right => k = 78.6 - 0.384 * desired_speed
			// before => leftMotSteps += (last_left_vel>>3);
			// now:
			leftMotSteps += ((float)(last_left_vel>>3))*(LEFT_ENC_OFFSET-ENC_SLOPE*((float)(last_left_vel>>2)))/1000.0;	// the result is directly in mm (divided by 1000 to get mm range)
		} else {
			leftMotSteps -= ((float)(last_left_vel>>3))*(LEFT_ENC_OFFSET-ENC_SLOPE*((float)(last_left_vel>>2)))/1000.0;
		}

*/

		start_speed_control_left(&pwm_left_working);

		pwm_left = pwm_left_working;

		if(pwm_left > 0) {
			OCR4A = (unsigned int)pwm_left;
		} else if(pwm_left < 0) {
			OCR4B =(unsigned int)( -pwm_left);
		} else {
			OCR4A = 0;
			OCR4B = 0;
		}

		computeOdometry++;

	}

	if(compute_right_vel) {

		last_right_vel = right_vel_sum>>2;
		compute_right_vel = 0;
		right_vel_sum = 0;

		if(calibrateOdomFlag==1) {
			rightSpeedSumOdom += last_right_vel;
			rightSumCount++;
		}

		getRightSpeedFromInput();
		rightDistPrev = rightDist;
		rightDist += ((float)speedRightFromEnc*((float)(getTime100MicroSec()-timeRightOdom)*104.0))/1000000.0;	// distance in mm				
		timeRightOdom = getTime100MicroSec();
		rightMotSteps = (signed long int)rightDist;

/*
		rightMotStepsOld = rightMotSteps;
		if(pwm_right_desired_to_control >= 0) {
			rightMotSteps += ((float)(last_right_vel>>3))*(RIGHT_ENC_OFFSET-ENC_SLOPE*((float)(last_right_vel>>2)))/1000.0;
		} else {
			rightMotSteps -= ((float)(last_right_vel>>3))*(RIGHT_ENC_OFFSET-ENC_SLOPE*((float)(last_right_vel>>2)))/1000.0;
		}
*/

		start_speed_control_right(&pwm_right_working);

		pwm_right = pwm_right_working;

		if(pwm_right > 0) {
			OCR3A = (unsigned int)pwm_right;
		} else if(pwm_right < 0) {
			OCR3B = (unsigned int)(-pwm_right);
		} else {
			OCR3A = 0;
			OCR3B = 0;
		}

		computeOdometry++;

	}

	if(computeOdometry>=2) {	// compute odometry when we get the last encoders values for both wheels

		// the odometry computation takes about 1 ms

		computeOdometry = 0;

		deltaDist = ((rightDist-rightDistPrev)+(leftDist-leftDistPrev))/2.0;

		if(robotPosition == HORIZONTAL_POS) {
			//thetaOld = (rightMotSteps - leftMotSteps)/WHEEL_DIST;	// radians
			theta = (rightDist-leftDist)/WHEEL_DIST;
		} else {
			//thetaOld = thetaAcc;
			theta = thetaAcc;
		}

		//deltaDistOld = ((rightMotSteps-rightMotStepsOld)+(leftMotSteps-leftMotStepsOld))/2.0;

		xPos = xPos + cos(theta)*deltaDist;				
		yPos = yPos + sin(theta)*deltaDist;

		//xPosOld = xPosOld + cos(thetaOld)*deltaDistOld;				
		//yPosOld = yPosOld + sin(thetaOld)*deltaDistOld;

	}

}

// vel expressed in 1/5 of mm/s
void setLeftSpeed(signed char vel) {

	speedl = abs(vel);

    if(vel >= 0) {
        pwm_left_desired = speedl;
    } else {
        pwm_left_desired = -(speedl);
    }

	if (pwm_left_desired>(MAX_MOTORS_PWM/2)) pwm_left_desired=(MAX_MOTORS_PWM/2);
	if (pwm_left_desired<-(MAX_MOTORS_PWM/2)) pwm_left_desired=-(MAX_MOTORS_PWM/2);

}

void setRightSpeed(signed char vel) {

	speedr = abs(vel);

    if(vel >= 0) {
        pwm_right_desired = speedr;
    } else {
        pwm_right_desired = -(speedr);
    }

	if (pwm_right_desired>(MAX_MOTORS_PWM/2)) pwm_right_desired=(MAX_MOTORS_PWM/2);
	if (pwm_right_desired<-(MAX_MOTORS_PWM/2)) pwm_right_desired=-(MAX_MOTORS_PWM/2);

}

void handleCalibration() {

	switch(calibState) {
		case CALIBRATION_STATE_FIND_THRS_0:
			timeoutOdometry = getTime100MicroSec();
			calibState = CALIBRATION_STATE_FIND_THRS_1;
			break;
		
		case CALIBRATION_STATE_FIND_THRS_1:	// Find the max and min of the ground sensor value in order to get a threshold to detect 
											// the black line securily (the threshold will be the average of the min and max).
			if((getTime100MicroSec() - timeoutOdometry)>PAUSE_1_SEC) { 	// Wait for the current sensor calibration to be terminated 
																		// (started when calibration is started).
				if(calibWheel == LEFT_WHEEL_FW_SC) {
					pwm_intermediate_right_desired = 0;
					pwm_intermediate_left_desired = (INDEX_STEP*3)<<2;		// Use a moderate speed.
	        	} else if(calibWheel == RIGHT_WHEEL_FW_SC) {
					pwm_intermediate_right_desired = (INDEX_STEP*3)<<2;
					pwm_intermediate_left_desired = 0;
				} else if(calibWheel == LEFT_WHEEL_BW_SC) {
					pwm_intermediate_right_desired = 0;
					pwm_intermediate_left_desired = -((INDEX_STEP*3)<<2);
	        	} else if(calibWheel == RIGHT_WHEEL_BW_SC) {
					pwm_intermediate_right_desired = -((INDEX_STEP*3)<<2);
					pwm_intermediate_left_desired = 0;
				}               
				minGround = 1023;
				maxGround = 0;
	            calibState = CALIBRATION_STATE_FIND_THRS_2;
	            timeoutOdometry = getTime100MicroSec();
			}
			break;

		case CALIBRATION_STATE_FIND_THRS_2:	// Wait for 5 seconds during which the ground min and max values are saved.
			if(calibWheel==LEFT_WHEEL_FW_SC || calibWheel==LEFT_WHEEL_BW_SC) {
				if(proximityResult[8] < minGround) {
					minGround = proximityResult[8];
				}
				if(proximityResult[8] > maxGround) {
					maxGround = proximityResult[8];
				}
			} else {
				if(proximityResult[11] < minGround) {
					minGround = proximityResult[11];
				}
				if(proximityResult[11] > maxGround) {
					maxGround = proximityResult[11];
				}
			}
			if((getTime100MicroSec() - timeoutOdometry)>PAUSE_5_SEC) {    // the robot seems to be still, go to next velcoity
				calibrationThr = (minGround + maxGround)>>1;	// Take the average of the 2 as the reference threshold value.
				//calibrationThrLow = calibrationThr - ((maxGround-minGround)>>2);	// Use an histeresys between max and min (not needed...).
				//calibrationThrHigh = calibrationThr + ((maxGround-minGround)>>2);
                calibState = CALIBRATION_STATE_SET_SPEED;
			}
			break;


    	case CALIBRATION_STATE_SET_SPEED: // set speed
        	if(calibWheel == LEFT_WHEEL_FW_SC) {
				pwm_intermediate_right_desired = 0;
				pwm_intermediate_left_desired = (INDEX_STEP*calibVelIndex)<<2;
        	} else if(calibWheel == RIGHT_WHEEL_FW_SC) {
				pwm_intermediate_right_desired = (INDEX_STEP*calibVelIndex)<<2;
				pwm_intermediate_left_desired = 0;
			} else if(calibWheel == LEFT_WHEEL_BW_SC) {
				pwm_intermediate_right_desired = 0;
				pwm_intermediate_left_desired = -((INDEX_STEP*calibVelIndex)<<2);
        	} else if(calibWheel == RIGHT_WHEEL_BW_SC) {
				pwm_intermediate_right_desired = -((INDEX_STEP*calibVelIndex)<<2);
				pwm_intermediate_left_desired = 0;
			}               
            calibState = CALIBRATION_STATE_START_MEASURE;
            timeoutOdometry = getTime100MicroSec();
            break;

		case CALIBRATION_STATE_START_MEASURE: // look for black line, start time measure
        	if(calibWheel==LEFT_WHEEL_FW_SC || calibWheel==LEFT_WHEEL_BW_SC) {
				if(proximityResult[8] < calibrationThr) {				
                	leftSumCount = 0;
                    leftSpeedSumOdom = 0;
                    timeOdometry = getTime100MicroSec();;
                    calibState = CALIBRATION_STATE_EXIT_BLACK_LINE_1;
                    timeoutOdometry = getTime100MicroSec();;
				}
			} else {
				if(proximityResult[11] < calibrationThr) {	
					rightSumCount = 0;
					rightSpeedSumOdom = 0;
                    timeOdometry = getTime100MicroSec();;
                    calibState = CALIBRATION_STATE_EXIT_BLACK_LINE_1;
                    timeoutOdometry = getTime100MicroSec();;
				}
			}
			if((getTime100MicroSec() - timeoutOdometry)>PAUSE_60_SEC) {    // the robot seems to be still, go to next velcoity
            	tempVel = 0;
				avgLeftSpeed = 0;
				avgRightSpeed = 0;
                updateOdomData();
                calibState = CALIBRATION_STATE_NEXT;
			}
			break;

		case CALIBRATION_STATE_EXIT_BLACK_LINE_1: // exit from black line
        	if(calibWheel==LEFT_WHEEL_FW_SC || calibWheel==LEFT_WHEEL_BW_SC) {
				if(proximityResult[8] > calibrationThr) {	
                	calibState = CALIBRATION_STATE_STOP_MEASURE;
                    timeoutOdometry = getTime100MicroSec();;
				}
			} else {
				if(proximityResult[11] > calibrationThr) {	
                	calibState = CALIBRATION_STATE_STOP_MEASURE;
                    timeoutOdometry = getTime100MicroSec();;
				}
			}
            if((getTime100MicroSec() - timeoutOdometry)>PAUSE_60_SEC) {    // the robot seems to be still, go to next velcoity
            	tempVel = 0;
				avgLeftSpeed = 0;
				avgRightSpeed = 0;
                updateOdomData();
                calibState = CALIBRATION_STATE_NEXT;
			}
            break;

		case CALIBRATION_STATE_STOP_MEASURE: // look for black line again, stop time measure
        	if(calibWheel==LEFT_WHEEL_FW_SC || calibWheel==LEFT_WHEEL_BW_SC) {
				if(proximityResult[8] < calibrationThr) {	
                	timeOdometry = getTime100MicroSec() - timeOdometry;
                    tempVel = (unsigned int)(DISTANCE_MM/((float)timeOdometry*104.0/1000000.0));
					avgLeftSpeed = leftSpeedSumOdom/leftSumCount;
                    updateOdomData();
                    calibState = CALIBRATION_STATE_EXIT_BLACK_LINE_2;
                    timeoutOdometry = getTime100MicroSec();;
				}
			} else {
				if(proximityResult[11] < calibrationThr) {	
                	timeOdometry = getTime100MicroSec() - timeOdometry;
                    tempVel = (unsigned int)(DISTANCE_MM/((float)timeOdometry*104.0/1000000.0));
                    avgRightSpeed = rightSpeedSumOdom/rightSumCount;
					updateOdomData();
                    calibState = CALIBRATION_STATE_EXIT_BLACK_LINE_2;
                    timeoutOdometry = getTime100MicroSec();;
				}
			}
			if((getTime100MicroSec() - timeoutOdometry)>PAUSE_60_SEC) {    // the robot seems to be still, go to next velcoity
            	tempVel = 0;
				avgLeftSpeed = 0;
				avgRightSpeed = 0;
                updateOdomData();
                calibState = CALIBRATION_STATE_NEXT;
			}
            break;

		case CALIBRATION_STATE_EXIT_BLACK_LINE_2: // exit from black line again
        	if(calibWheel==LEFT_WHEEL_FW_SC || calibWheel==LEFT_WHEEL_BW_SC) {
				if(proximityResult[8] > calibrationThr) {	
                	calibState = CALIBRATION_STATE_NEXT;
				}
			} else {
				if(proximityResult[11] > calibrationThr) {	
					calibState = CALIBRATION_STATE_NEXT;
				}
			}
            if((getTime100MicroSec() - timeoutOdometry)>PAUSE_60_SEC) {    // the robot seems to be still, go to next velocity
            	tempVel = 0;
                //updateOdomData();
                calibState = CALIBRATION_STATE_NEXT;
			}
            break;

		case CALIBRATION_STATE_NEXT:
        	calibVelIndex++;
            if(calibVelIndex == 10) {
            	calibVelIndex = 1;
                if(calibWheel == LEFT_WHEEL_FW_SC) {
                	calibWheel = LEFT_WHEEL_BW_SC;
				} else if(calibWheel == RIGHT_WHEEL_FW_SC) {
                	calibWheel = RIGHT_WHEEL_BW_SC;
				} else if(calibWheel == LEFT_WHEEL_BW_SC) {
                	calibWheel = RIGHT_WHEEL_FW_SC;
					calibrateOdomFlag = 0;
					calibState = CALIBRATION_STATE_FIND_THRS_0;	// Recompute the thresholds for the right ground.
					break;
					// red on
				} else if(calibWheel == RIGHT_WHEEL_BW_SC) {
                	calibWheel = LEFT_WHEEL_FW_SC;					
					// red off
					writeCalibrationToFlash();
					calibrateOdomFlag = 0;
				}
			}
			calibState = CALIBRATION_STATE_SET_SPEED;
			break;

		default:
        	break;

	}


}


void updateOdomData() {

    if(calibWheel == LEFT_WHEEL_FW_SC) {
        if(calibVelIndex>1) {
            if(calibration[calibVelIndex-2][1] >= tempVel) {  // check that we have always increasing values of speed, otherwise there
                tempVel = calibration[calibVelIndex-2][1]+1; // will be problems when getting data from the lookup table
            }
        }
        calibration[calibVelIndex-1][0] = avgLeftSpeed;
        calibration[calibVelIndex-1][1] = tempVel;
    } else if(calibWheel == RIGHT_WHEEL_FW_SC) {
        if(calibVelIndex>1) {
            if(calibration[calibVelIndex-2][3] >= tempVel) {
                tempVel = calibration[calibVelIndex-2][3]+1;
            }
        }
		calibration[calibVelIndex-1][2] = avgRightSpeed;
        calibration[calibVelIndex-1][3] = tempVel;
    } else if(calibWheel == LEFT_WHEEL_BW_SC) {
        if(calibVelIndex>1) {
            if(calibration[calibVelIndex-2][5] >= tempVel) {
                tempVel = calibration[calibVelIndex-2][5]+1;
            }
        }
		calibration[calibVelIndex-1][4] = avgLeftSpeed;
        calibration[calibVelIndex-1][5] = tempVel;
    } else if(calibWheel == RIGHT_WHEEL_BW_SC) {
        if(calibVelIndex>1) {
            if(calibration[calibVelIndex-2][7] >= tempVel) {
                tempVel = calibration[calibVelIndex-2][7]+1;
            }
        }
		calibration[calibVelIndex-1][6] = avgRightSpeed;
        calibration[calibVelIndex-1][7] = tempVel;
    }

}

// extract data to pass to speed controller given a desired speed in mm/s
// mode => return a measured speed 0..1023
signed int getInputFromSpeed(signed int s, unsigned char mode) {
    
    int i = 0;
    signed int currVel = s*BYTE_TO_MM_S;
    signed int temp = 0;

    if(currVel == 0) {
        return 0;
    }

    if(mode==LEFT_WHEEL_BW_SC || mode==RIGHT_WHEEL_BW_SC) {
        currVel = -currVel; // consider only positive values
    }

    for(i=0; i<CALIBRATION_SAMPLES; i++) {
        if(mode==LEFT_WHEEL_FW_SC) {
            if(calibration[i][1] >= currVel) {
                break;
            }
        } else if(mode==RIGHT_WHEEL_FW_SC) {
            if(calibration[i][3] >= currVel) {
                break;
            }
        } else if(mode==LEFT_WHEEL_BW_SC) {
            if(calibration[i][5] >= currVel) {
                break;
            }
        } else if(mode==RIGHT_WHEEL_BW_SC) {
            if(calibration[i][7] >= currVel) {
                break;
            }
        } 
    }

    if(i==0) {  // the velocity is lower than first saved in the matrix
        if(mode==LEFT_WHEEL_FW_SC) {
            temp = (currVel*calibration[0][0])/calibration[0][1];
        } else if(mode==RIGHT_WHEEL_FW_SC) {
            temp = (currVel*calibration[0][2])/calibration[0][3];
        } else if(mode==LEFT_WHEEL_BW_SC) {
            temp = currVel*calibration[0][4]/calibration[0][5];
            temp = -temp;
        } else if(mode==RIGHT_WHEEL_BW_SC) {
            temp = currVel*calibration[0][6]/calibration[0][7];
            temp = -temp;
        }        
    } else if(i==CALIBRATION_SAMPLES) {   // the velocity is greater than all the ones saved in the matrix
        if(mode==LEFT_WHEEL_FW_SC) {
            temp = (signed int)((float)currVel*(float)calibration[CALIBRATION_SAMPLES-1][0]/(float)calibration[CALIBRATION_SAMPLES-1][1]);
	    } else if(mode==RIGHT_WHEEL_FW_SC) {
            temp = (signed int)((float)currVel*(float)calibration[CALIBRATION_SAMPLES-1][2]/(float)calibration[CALIBRATION_SAMPLES-1][3]);
        } else if(mode==LEFT_WHEEL_BW_SC) {
            temp = (signed int)((float)currVel*(float)calibration[CALIBRATION_SAMPLES-1][4]/(float)calibration[CALIBRATION_SAMPLES-1][5]);
            temp = -temp;
        } else if(mode==RIGHT_WHEEL_BW_SC) {
            temp = (signed int)((float)currVel*(float)calibration[CALIBRATION_SAMPLES-1][6]/(float)calibration[CALIBRATION_SAMPLES-1][7]);
            temp = -temp;
        }
    } else {
        if(mode==LEFT_WHEEL_FW_SC) {
            temp = calibration[i-1][0] + (signed int)(((float)(currVel-calibration[i-1][1])*(float)(calibration[i][0]-calibration[i-1][0]))/(float)(calibration[i][1]-calibration[i-1][1]));
        } else if(mode==RIGHT_WHEEL_FW_SC) {
            temp = calibration[i-1][2] + (signed int)(((float)(currVel-calibration[i-1][3])*(float)(calibration[i][2]-calibration[i-1][2]))/(float)(calibration[i][3]-calibration[i-1][3]));
        } else if(mode==LEFT_WHEEL_BW_SC) {
            temp = calibration[i-1][4] + (signed int)(((float)(currVel-calibration[i-1][5])*(float)(calibration[i][4]-calibration[i-1][4]))/(float)(calibration[i][5]-calibration[i-1][5]));
            temp = -temp;
        } else if(mode==RIGHT_WHEEL_BW_SC) {
            temp = calibration[i-1][6] + (signed int)(((float)(currVel-calibration[i-1][7])*(float)(calibration[i][6]-calibration[i-1][6]))/(float)(calibration[i][7]-calibration[i-1][7]));
            temp = -temp;
        }        
    }
    
    return temp;
}

// extract the speed of the motors in mm/s given a measured speed (adc)
void getRightSpeedFromInput() {

    signed int i=0, indFwR=-1, indBwR=-1;
    
    for(i=0; i<CALIBRATION_SAMPLES; i++) {
		if(pwm_right >= 0) {
			if(calibration[i][2]>=last_right_vel && indFwR<0) {	// forward right
				indFwR = i;
			}
		} else {
			if(calibration[i][6]>=last_right_vel && indBwR<0) {	// backward right
				indBwR = i;
			}
		}        
    }

    if(pwm_right >= 0) {
        if(last_right_vel == 0) {
            speedRightFromEnc = 0;
        } else {
            if(indFwR==0) {  // the velocity is lower than first saved in the matrix
                speedRightFromEnc = (last_right_vel*calibration[0][3])/calibration[0][2];
            } else if(indFwR==-1) { //CALIBRATION_SAMPLES) {   // the velocity is greater than all the ones saved in the matrix
                speedRightFromEnc = (signed int)(((float)calibration[CALIBRATION_SAMPLES-1][3]*(float)last_right_vel)/(float)calibration[CALIBRATION_SAMPLES-1][2]);  // take the max
            } else {
                speedRightFromEnc = calibration[indFwR-1][3] + (signed int)(((float)(last_right_vel-calibration[indFwR-1][2])*(float)(calibration[indFwR][3]-calibration[indFwR-1][3]))/(float)(calibration[indFwR][2]-calibration[indFwR-1][2]));
            }
        }
    } else {
		if(indBwR==0) {  // the velocity is lower than first saved in the matrix
        	speedRightFromEnc = (last_right_vel*calibration[0][7])/calibration[0][6];
		} else if(indBwR==-1) { //CALIBRATION_SAMPLES) {   // the velocity is greater than all the ones saved in the matrix
        	speedRightFromEnc = (signed int)(((float)calibration[CALIBRATION_SAMPLES-1][7]*(float)last_right_vel)/(float)calibration[CALIBRATION_SAMPLES-1][6]);  // take the max
		} else {
        	speedRightFromEnc = calibration[indBwR-1][7] + (signed int)(((float)(last_right_vel-calibration[indBwR-1][6])*(float)(calibration[indBwR][7]-calibration[indBwR-1][7]))/(float)(calibration[indBwR][6]-calibration[indBwR-1][6]));
		}
		speedRightFromEnc = -speedRightFromEnc;
    }
    
}

// extract the speed of the motors in mm/s given a measured speed (adc)
void getLeftSpeedFromInput() {

    signed int i=0, indFwL=-1, indBwL=-1;
    
    for(i=0; i<CALIBRATION_SAMPLES; i++) {
		if(pwm_left >= 0) {
			if(calibration[i][0]>=last_left_vel && indFwL<0) {	// forward left
				indFwL = i;
			}
		} else {
			if(calibration[i][4]>=last_left_vel && indBwL<0) {	// backward left
				indBwL = i;
			}
		}     
    }

    if(pwm_left >= 0) {
        if(last_left_vel == 0) {
            speedLeftFromEnc = 0;
        } else {
            if(indFwL==0) {  // the velocity is lower than first saved in the matrix
                speedLeftFromEnc = (last_left_vel*calibration[0][1])/calibration[0][0];
            } else if(indFwL==-1) { //CALIBRATION_SAMPLES) {   // the velocity is greater than all the ones saved in the matrix
                speedLeftFromEnc = (signed int)(((float)calibration[CALIBRATION_SAMPLES-1][1]*(float)last_left_vel)/(float)calibration[CALIBRATION_SAMPLES-1][0]);  // take the max
            } else {
                speedLeftFromEnc = calibration[indFwL-1][1] + (signed int)(((float)(last_left_vel-calibration[indFwL-1][0])*(float)(calibration[indFwL][1]-calibration[indFwL-1][1]))/(float)(calibration[indFwL][0]-calibration[indFwL-1][0]));
            }
        }
    } else {
		if(indBwL==0) {  // the velocity is lower than first saved in the matrix
        	speedLeftFromEnc = (last_left_vel*calibration[0][5])/calibration[0][4];
		} else if(indBwL==-1) { //CALIBRATION_SAMPLES) {   // the velocity is greater than all the ones saved in the matrix
        	speedLeftFromEnc = (signed int)(((float)calibration[CALIBRATION_SAMPLES-1][5]*(float)last_left_vel)/(float)calibration[CALIBRATION_SAMPLES-1][4]);  // take the max
		} else {
        	speedLeftFromEnc = calibration[indBwL-1][5] + (signed int)(((float)(last_left_vel-calibration[indBwL-1][4])*(float)(calibration[indBwL][5]-calibration[indBwL-1][5]))/(float)(calibration[indBwL][4]-calibration[indBwL-1][4]));
		}
		speedLeftFromEnc = -speedLeftFromEnc;
    }
    
}

void writeDefaultCalibration() {
	int i = 0;
    for(i=0; i<CALIBRATION_SAMPLES; i++) {
    	calibration[i][0] = i+1;
	}
    // the following values are taken from a field test
    // forward left, speed control enabled
    calibration[0][0] = 20;		// measured speed with back EMF (adc 0..1023)
    calibration[1][0] = 40;
    calibration[2][0] = 59;
    calibration[3][0] = 79;
    calibration[4][0] = 100;
    calibration[5][0] = 120;
    calibration[6][0] = 139;
    calibration[7][0] = 159;
	calibration[8][0] = 180;
    calibration[0][1] = 19;    	// real speed measured in mm/s
    calibration[1][1] = 47;
    calibration[2][1] = 72;
    calibration[3][1] = 91;
    calibration[4][1] = 109;
    calibration[5][1] = 125;
    calibration[6][1] = 143;
    calibration[7][1] = 160;
	calibration[8][1] = 174;
    // forward right, speed control enabled
    calibration[0][2] = 19;		// measured speed with back EMF (adc 0..1023)
    calibration[1][2] = 39;
    calibration[2][2] = 60;
    calibration[3][2] = 80;
    calibration[4][2] = 99;
    calibration[5][2] = 119;
    calibration[6][2] = 140;
    calibration[7][2] = 160;
	calibration[8][2] = 180;
    calibration[0][3] = 23;    	// real speed measured in mm/s
    calibration[1][3] = 46;
    calibration[2][3] = 64;
    calibration[3][3] = 81;
    calibration[4][3] = 97;
    calibration[5][3] = 112;
    calibration[6][3] = 125;
    calibration[7][3] = 139;
	calibration[8][3] = 152;
    // backward left, speed control enabled
    calibration[0][4] = 20;		// measured speed with back EMF (adc 0..1023)
    calibration[1][4] = 39;
    calibration[2][4] = 60;
    calibration[3][4] = 79;
    calibration[4][4] = 99;
    calibration[5][4] = 120;
    calibration[6][4] = 140;
    calibration[7][4] = 160;
	calibration[8][4] = 179;
    calibration[0][5] = 18;    	// real speed measured in mm/s
    calibration[1][5] = 45;
    calibration[2][5] = 68;
    calibration[3][5] = 87;
    calibration[4][5] = 105;
    calibration[5][5] = 119;
    calibration[6][5] = 137;
    calibration[7][5] = 151;
	calibration[8][5] = 168;
    // backward right, speed control enabled
    calibration[0][6] = 20;		// measured speed with back EMF (adc 0..1023)
    calibration[1][6] = 39;
    calibration[2][6] = 59;
    calibration[3][6] = 80;
    calibration[4][6] = 100;
    calibration[5][6] = 119;
    calibration[6][6] = 139;
    calibration[7][6] = 160;
	calibration[8][6] = 180;
    calibration[0][7] = 22;    	// real speed measured in mm/s
    calibration[1][7] = 46;
    calibration[2][7] = 65;
    calibration[3][7] = 81;
    calibration[4][7] = 96;
    calibration[5][7] = 111;
    calibration[6][7] = 125;
    calibration[7][7] = 139;
	calibration[8][7] = 153;

    writeCalibrationToFlash();
}

void initCalibration() {

    unsigned int temp=0;
 
 	temp = eeprom_read_word((uint16_t*)CALIB_CHECK_ADDRESS);

    if(temp==0xAA55) {   // valid odometry data saved in flash, read them
        readCalibrationFromFlash();
    } else {
		writeDefaultCalibration();		
    }

}

// Handle "soft acceleration" that basically increase or decrease the current speed
// at steps untill raching the new desired speed, resulting in a smooth acceleration).
// At the moment the "soft acceleration" is disabled and can be enabled only in the code 
// by setting the value of "softAccEnabled" to 1 at variable initialization.
void handleSoftAcceleration() {
		
	if(calibrateOdomFlag==0) {
		if((getTime100MicroSec()-speedStepCounter) >= SPEED_STEP_DELAY) {
			speedStepCounter = getTime100MicroSec();

			if(softAccEnabled) {
				if(pwm_right_desired == 0) {
					pwm_intermediate_right_desired = 0;
				} else if((pwm_right_desired*pwm_intermediate_right_desired) < 0) {
					pwm_intermediate_right_desired = 0;
				} else if(pwm_right_desired > pwm_intermediate_right_desired) {
					pwm_intermediate_right_desired += speedStep;
					if(pwm_intermediate_right_desired > pwm_right_desired) {
						pwm_intermediate_right_desired = pwm_right_desired;
					}
				} else if(pwm_right_desired < pwm_intermediate_right_desired) {
					pwm_intermediate_right_desired -= speedStep;
					if(pwm_intermediate_right_desired < pwm_right_desired) {
						pwm_intermediate_right_desired = pwm_right_desired;
					}					
				}
	
				if(pwm_left_desired == 0) {
					pwm_intermediate_left_desired = 0;
				} else if((pwm_left_desired*pwm_intermediate_left_desired) < 0) {
					pwm_intermediate_left_desired = 0;
				} else if(pwm_left_desired > pwm_intermediate_left_desired) {
					pwm_intermediate_left_desired += speedStep;
					if(pwm_intermediate_left_desired > pwm_left_desired) {
						pwm_intermediate_left_desired = pwm_left_desired;
					}
				} else if(pwm_left_desired < pwm_intermediate_left_desired) {
					pwm_intermediate_left_desired -= speedStep;
					if(pwm_intermediate_left_desired < pwm_left_desired) {
						pwm_intermediate_left_desired = pwm_left_desired;
					}					
				}
			} else {
				pwm_intermediate_right_desired = pwm_right_desired;
				pwm_intermediate_left_desired = pwm_left_desired;
			}

		}
	}

}


// Motor left
ISR(TIMER4_OVF_vect) {

//	LED_GREEN_ON;

	if(cliffDetectedFlag) {
		pwm_left = 0;
		OCR4A = 0;
		OCR4B = 0;
	}

	left_current_avg = 0;

	// set pins mode based on controller output
	if(pwm_left == 0) {
		//firstSampleLeft = 0;


		//leftMotorPhase = NO_PHASE;
		//compute_left_vel = 1;

		if(pwm_left_desired_to_control >= 0) {
			leftMotorPhase = PASSIVE_PHASE;
			currentMotLeftChannel = 14;
		} else {
			leftMotorPhase = PASSIVE_PHASE;
			currentMotLeftChannel = 15;
		}
		firstSampleLeft = 1;

		// select channel 15 to sample left current
		//currentMotLeftChannel = 15;
		TCCR4A  &= ~(1 << COM4A1) & ~(1 << COM4B1);	// disable OCA and OCB
		PORTH &= ~(1 << 4) & ~(1 << 3);				// output to 0
		TIMSK4 &= ~(1 << OCIE4B) & ~(1 << OCIE4A);	// disable OCA and OCB interrupt
		//TIMSK4 |= (1 << OCIE4A);		// enable OCA interrupt => sampling of velocity is enabled even if
										// the pwm is turned off...is it correct??
		TIFR4 |= (1 << OCF4A) | (1 << OCF4B);
	} else if(pwm_left > 0) {   		// move forward
		leftMotorPhase = ACTIVE_PHASE;
		// select channel 15 to sample left current
		currentMotLeftChannel = 15;
		TCCR4A  &= ~(1 << COM4B1);		// disable OCB
		TIMSK4 &= ~(1 << OCIE4B);		// disable OCB interrupt
		PORTH &= ~(1 << 4);				// output to 0
		TCCR4A |= (1 << COM4A1);		// enable OCA
		TIMSK4 |= (1 << OCIE4A);		// enable OCA interrupt
	} else if(pwm_left < 0) {      		// move backward
		leftMotorPhase = ACTIVE_PHASE;
		// select channel 14 to sample left current
		currentMotLeftChannel = 14;
		TCCR4A  &= ~(1 << COM4A1);		// disable OCA
		TIMSK4 &= ~(1 << OCIE4A);		// disable OCA interrupt
		PORTH &= ~(1 << 3);				// output to 0
		TCCR4A |= (1 << COM4B1);		// enable OCB
		TIMSK4 |= (1 << OCIE4B);		// enable OCB interrupt
	}

/*
	// set channel to sample based on desired velocity (direction);
	// we cannot rely on the sole controller output value to decide which
	// channel to sample due to the "zero" special case: there are basically
	// two zeros, one for the forward direction and one for the backward.
	// The controller thus can output zero both when running forward and backward
	// but the correct channel to sample is different in each of the two cases.
	// Thus it's better to check the desired velocity that gives us the direction
	// of the robot (the controller cannot change the direction) and consequently
	// the correct channel to sample.
	if(pwm_left_desired >= 0) {
		leftMotorPhase = ACTIVE_PHASE;
		currentMotLeftChannel = 15;
	} else {
		leftMotorPhase = ACTIVE_PHASE;
		currentMotLeftChannel = 14;
	}
*/

//	LED_GREEN_OFF;

}

// motor left forward
ISR(TIMER4_COMPA_vect) {

//	LED_GREEN_ON;

//	if(pwm_left == 0) {
//		return;
//	}

	leftMotorPhase = PASSIVE_PHASE;
	// select channel 14 to sample the left velocity
	currentMotLeftChannel = 14;

	firstSampleLeft = 1;

//	LED_GREEN_OFF;

}

// motor left backward
ISR(TIMER4_COMPB_vect) {

//	LED_GREEN_ON;

//	if(pwm_left == 0) {
//		return;
//	}

	leftMotorPhase = PASSIVE_PHASE;
	// select channel 15 to sample the left velocity
	currentMotLeftChannel = 15;

	firstSampleLeft = 1;

//	LED_GREEN_OFF;

}

// Motor right
ISR(TIMER3_OVF_vect) {

//	LED_GREEN_ON;

  	// PORTB ^= (1 << 7); // Toggle the LED

	if(cliffDetectedFlag) {
		pwm_right = 0;
		OCR3A = 0;
		OCR3B = 0;
	}

	right_current_avg = 0;


	if(pwm_right == 0) {
		//firstSampleRight = 0;

		//rightMotorPhase = NO_PHASE;
		//compute_right_vel = 1;

		if(pwm_right_desired_to_control >= 0) {
			rightMotorPhase = PASSIVE_PHASE;
			// select channel 13 to sample left current
			currentMotRightChannel = 12;
		} else {
			rightMotorPhase = PASSIVE_PHASE;
			// select channel 12 to sample left current
			currentMotRightChannel = 13;
		}
		firstSampleRight = 1;

		// select channel 13 to sample left current
		//currentMotRightChannel = 13;
		TCCR3A  &= ~(1 << COM3A1) & ~(1 << COM3B1);	// disable OCA and OCB
		PORTE &= ~(1 << 4) & ~(1 << 3);				// output to 0
		TIMSK3 &= ~(1 << OCIE3B) & ~(1 << OCIE3A);	// disable OCA and OCB interrupt
		//TIMSK3 |= (1 << OCIE3A);		// enable OCA interrupt => sampling of velocity is enabled even if
										// the pwm is turned off...is it correct??
		TIFR3 |= (1 << OCF3A) | (1 << OCF3B);
	}else if(pwm_right > 0) {   		// move forward
		rightMotorPhase = ACTIVE_PHASE;
		// select channel 13 to sample left current
		currentMotRightChannel = 13;
		TCCR3A  &= ~(1 << COM3B1);		// disable OCB
		TIMSK3 &= ~(1 << OCIE3B);		// disable OCB interrupt
		PORTE &= ~(1 << 4);				// output to 0
		TCCR3A |= (1 << COM3A1);		// enable OCA
		TIMSK3 |= (1 << OCIE3A);		// enable OCA interrupt
	} else if(pwm_right < 0) {      	// move backward
		rightMotorPhase = ACTIVE_PHASE;
		// select channel 12 to sample left current
		currentMotRightChannel = 12;
		TCCR3A  &= ~(1 << COM3A1);		// disable OCA
		TIMSK3 &= ~(1 << OCIE3A);		// disable OCA interrupt
		PORTE &= ~(1 << 3);				// output to 0
		TCCR3A |= (1 << COM3B1);		// enable OCB
		TIMSK3 |= (1 << OCIE3B);		// enable OCB interrupt
	}

/*
	if(pwm_right_desired >= 0) {
		rightMotorPhase = ACTIVE_PHASE;
		// select channel 13 to sample left current
		currentMotRightChannel = 13;
	} else {
		rightMotorPhase = ACTIVE_PHASE;
		// select channel 12 to sample left current
		currentMotRightChannel = 12;
	}
*/
//	LED_GREEN_OFF;

}

// motor right forward
ISR(TIMER3_COMPA_vect) {

//	LED_RED_ON;

//	if(pwm_right == 0) {
//		return;
//	}

	rightMotorPhase = PASSIVE_PHASE;
	// select channel 12 to sample the right velocity
	currentMotRightChannel = 12;

	firstSampleRight = 1;

//	LED_RED_OFF;

}

// motor right backward
ISR(TIMER3_COMPB_vect) {

//	LED_RED_ON;

//	if(pwm_right == 0) {
//		return;
//	}

	rightMotorPhase = PASSIVE_PHASE;
	// select channel 13 to sample the right velocity
	currentMotRightChannel = 13;

	firstSampleRight = 1;

//	LED_RED_OFF;
}
