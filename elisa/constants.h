#include <avr\io.h>

/***************/
/*** VARIOUS ***/
/***************/
#define MAX_U32 4294967295

#ifndef RAD_2_DEG
#define RAD_2_DEG 57.2957796				// conversion factor from radiant to degrees; 
#endif										// use: degrees_value = radiant_value * RAD_2_DEG

#ifndef CALIBRATION_CYCLES
#define CALIBRATION_CYCLES 16				// number of samples used for calibration
#endif

#ifndef PAUSES								// pauses based on adc base time counter "delayCounter"
#define PAUSE_1_MSEC 9						// they are not perfectly precise (based on 104 us)
#define PAUSE_10_MSEC 96
#define PAUSE_100_MSEC 961
#define PAUSE_200_MSEC 1923
#define PAUSE_250_MSEC 2404
#define PAUSE_300_MSEC 2885
#define PAUSE_400_MSEC 3846
#define PAUSE_500_MSEC 4807
#define PAUSE_750_MSEC 7211
#define PAUSE_1_SEC 9615
#define PAUSE_1500_MSEC 14423
#define PAUSE_2_SEC 19230
#define PAUSE_4_SEC 38462
#define PAUSE_5_SEC 48077
#define PAUSE_6_SEC 57692
#define PAUSE_10_SEC 96154
#define PAUSE_15_SEC 144230
#define PAUSE_20_SEC 192307
#define PAUSE_30_SEC 288461
#define PAUSE_40_SEC 384615
#define PAUSE_60_SEC 576923
#endif

#ifndef UART_BUFF_SIZE
#define UART_BUFF_SIZE 256
#endif

#define LINE_IN_THR 400
#define LINE_OUT_THR 450

#define SPEED_STEP_DELAY 400					// based on adc interrupt that is about 104 us
											// speed updated every SPEED_STEP_FREQ*104 us untill desired
											// speed reached

/***********/
/*** NRF ***/
/***********/
#ifndef PAYLOAD_SIZE
#define PAYLOAD_SIZE 13						// payload of the packet received from the computer
#endif

/************************/
/*** SPEED CONTROLLER ***/
/************************/
#ifndef PERIOD_MOTORS_100HZ
#define PERIOD_MOTORS_100HZ	40000			// old controller (elisa 2): we need a period time of 10 ms (100 Hz) => 0.01 * 4000000 = 40000
#endif

#ifndef MAX_PWM
#define MAX_PWM PERIOD_MOTORS_100HZ/100*60	// old controller (elisa 2): 60% of maximum
#endif

#ifndef MAX_MOTORS_PWM
#define MAX_MOTORS_PWM 1023					// max value for the pwm registers of the motors; 50% of maximum is then used actually
#endif

// horizontal speed controller
#ifndef P_PART
#define P_PART 10 //15						// P parameter of the PID speed controller
#endif

#ifndef I_PART
#define I_PART 3 //2						// I parameter of the PID speed controller
#endif

#ifndef D_PART
#define D_PART 2 //3						// D parameter of the PID speed controller
#endif

#ifndef I_LIMIT
#define I_LIMIT 1600				// limit of the errors sum
#endif

/****************/
/*** RGB LEDS ***/
/****************/
#ifndef MAX_LEDS_PWM
#define MAX_LEDS_PWM 255					// max value for the pwm registers of the leds
#endif

/*************************/
/*** IR REMOTE CONTROL ***/
/*************************/
#ifndef STEP_MOTORS
#define STEP_MOTORS 2						// step used to increase/decrease the motors speed when 
#endif										// receiving a command through TV remote.
											// speed (0..127) is expressed in 1/5 of mm/s (0..635 mm/s), thus 
											// 2 means steps of 10 mm/s

/*********************/
/*** ACCELEROMETER ***/
/*********************/
#ifndef VERTICAL_THRESHOLD
#define VERTICAL_THRESHOLD 45				// threshold value to swtich from horizontal to vertical plane
#endif										// (when Z > threshold then horizontal plane)

#ifndef SAME_POS_NUM
#define SAME_POS_NUM 200						// hysteresis value to pass from horizontal to vertical plane and
#endif										// viceversa.

#ifndef VERTICAL_POS
#define VERTICAL_POS 0						// for easier code comprehension
#endif

#ifndef HORIZONTAL_POS
#define HORIZONTAL_POS 1					// for easier code comprehension
#endif

#ifndef MMA7455L_ADDR
#define MMA7455L_ADDR (0x1D<<1)				// address to communicate with the MMA7455L
#endif

#ifndef ADXL345_ADDR
#define ADXL345_ADDR (0x53<<1)				// address to communicate with the ADXL345
#endif

#ifndef USE_MMAX7455L
#define USE_MMAX7455L 0						// for easier code comprehension
#endif

#ifndef USE_ADXL345
#define USE_ADXL345 1						// for easier code comprehension
#endif

#ifndef USE_NO_ACCEL
#define USE_NO_ACCEL 2						// for easier code comprehension
#endif

/*********************/
/*** HARDWARE DEFS ***/
/*********************/
#ifndef HW_REV
	#define HW_REV_3_0 0
	#define HW_REV_3_0_1 1
	#define HW_REV_3_1 2
#endif

#ifndef REMOTE
#define REMOTE (PINJ & _BV(PJ6))>>6			// tv remote pin state
#endif

#ifndef SEL0
#define SEL0 (PINC & _BV(PC0))>>0			// selector pins state
#endif

#ifndef SEL1
#define SEL1 (PINC & _BV(PC1))>>1
#endif

#ifndef SEL2
#define SEL2 (PINC & _BV(PC2))>>2
#endif

#ifndef SEL3
#define SEL3 (PINC & _BV(PC3))>>3
#endif

#ifndef CHARGE_ON
#define CHARGE_ON ((PINJ & _BV(PJ4))>>4)	// charge_on pin state: 0 => robot not in charge; 1 => robot in charge
#endif

#ifndef BUTTON0
#define BUTTON0 ((PINJ & _BV(PJ5))>>5)		// button pin state: 0 => button pressed; 1 => button not pressed
#endif

#ifndef CHARGE_STAT
#define CHARGE_STAT ((PINJ & _BV(PJ7))>>7)	// charge_stat pin state: 0 => robot not charged completely; 1 => robot charged completely 
#endif

#ifndef SENS_ENABLE_OFF
#define SENS_ENABLE_OFF (PORTC &= ~(1 << 6))// used for sampling the battery
#endif

#ifndef SENS_ENABLE_ON
#define SENS_ENABLE_ON (PORTC |= (1 << 6))
#endif

#ifndef LED_IR1_HIGH
#define LED_IR1_HIGH (PORTC |= (1<<4))		// turn off backward IR
#endif

#ifndef LED_IR1_LOW
#define LED_IR1_LOW (PORTC &= ~(1<<4))		// turn on backward IR
#endif

#ifndef LED_IR2_HIGH
#define LED_IR2_HIGH (PORTC |= (1<<5))		// turn off both forward IRs
#endif
 
#ifndef LED_IR2_LOW
#define LED_IR2_LOW (PORTC &= ~(1<<5))		// turn on both forward IRs
#endif

// the LED_X_ON/OFF macros works only if the leds port pins aren't
// set in pwm mode
#ifndef LED_RED_OFF
#define LED_RED_OFF (PORTB |= (1<<5))
#endif

#ifndef LED_GREEN_OFF
#define LED_GREEN_OFF (PORTB |= (1<<6))
#endif

#ifndef LED_BLUE_OFF
#define LED_BLUE_OFF (PORTB |= (1<<7))
#endif

#ifndef LED_RED_ON
#define LED_RED_ON (PORTB &= ~(1<<5))
#endif

#ifndef LED_GREEN_ON
#define LED_GREEN_ON (PORTB &= ~(1<<6))
#endif

#ifndef LED_BLUE_ON
#define LED_BLUE_ON (PORTB &= ~(1<<7))
#endif

// small green leds handling macros
#ifndef GREEN_LEDS
#define GREEN_LED0_ON (PORTL &= ~(1 << 0))
#define GREEN_LED1_ON (PORTL &= ~(1 << 1))
#define GREEN_LED2_ON (PORTL &= ~(1 << 2))
#define GREEN_LED3_ON (PORTG &= ~(1 << 3))
#define GREEN_LED4_ON (PORTL &= ~(1 << 4))
#define GREEN_LED5_ON (PORTL &= ~(1 << 5))
#define GREEN_LED6_ON (PORTL &= ~(1 << 6))
#define GREEN_LED7_ON (PORTL &= ~(1 << 7))
#define GREEN_LED0_OFF (PORTL |= (1 << 0))
#define GREEN_LED1_OFF (PORTL |= (1 << 1))
#define GREEN_LED2_OFF (PORTL |= (1 << 2))
#define GREEN_LED3_OFF (PORTG |= (1 << 3))
#define GREEN_LED4_OFF (PORTL |= (1 << 4))
#define GREEN_LED5_OFF (PORTL |= (1 << 5))
#define GREEN_LED6_OFF (PORTL |= (1 << 6))
#define GREEN_LED7_OFF (PORTL |= (1 << 7))
#endif

/***********/
/*** ADC ***/
/***********/
#ifndef ACTIVE_PHASE
#define ACTIVE_PHASE 0						// for easier code comprehension: indicate the active phase of the motors pwm
#endif

#ifndef PASSIVE_PHASE
#define PASSIVE_PHASE 1						// for easier code comprehension: indicate the passive phase of the motors pwm
#endif

#ifndef NO_PHASE
#define NO_PHASE 2							// currently no more used (when pwm = 0)
#endif

#ifndef SAVE_TO_PROX
#define SAVE_TO_PROX 0						// for easier code comprehension: indicate that the value currently sampled has 
#endif										// to be saved in the proximity array

#ifndef SAVE_TO_RIGHT_MOTOR_CURRENT
#define SAVE_TO_RIGHT_MOTOR_CURRENT 1		// for easier code comprehension: current sample is current consumption for right motor
#endif

#ifndef SAVE_TO_RIGHT_MOTOR_VEL
#define SAVE_TO_RIGHT_MOTOR_VEL 2			// for easier code comprehension: current sample is velocity for right motor
#endif

#ifndef SAVE_TO_LEFT_MOTOR_CURRENT
#define SAVE_TO_LEFT_MOTOR_CURRENT 3		// for easier code comprehension: current sample is current consumption for left motor
#endif

#ifndef SAVE_TO_LEFT_MOTOR_VEL
#define SAVE_TO_LEFT_MOTOR_VEL 4			// for easier code comprehension: current sample is velocity for left motor
#endif

#ifndef SKIP_SAMPLE
#define SKIP_SAMPLE 5						// currently no more needed
#endif

#ifndef SAVE_TO_PROX_IRCOMM
#define SAVE_TO_PROX_IRCOMM 6
#endif

/***************/
/*** SENSORS ***/
/***************/
#ifndef PROX0_AMBIENT						// array indices for the sensors for easier code comprehension
#define PROX0_AMBIENT 0
#define PROX0_REFLECTED 1
#define PROX1_AMBIENT 2
#define PROX1_REFLECTED 3
#define PROX2_AMBIENT 4
#define PROX2_REFLECTED 5
#define PROX3_AMBIENT 6
#define PROX3_REFLECTED 7
#define PROX4_AMBIENT 8
#define PROX4_REFLECTED 9
#define PROX5_AMBIENT 10
#define PROX5_REFLECTED 11
#define PROX6_AMBIENT 12
#define PROX6_REFLECTED 13
#define PROX7_AMBIENT 14
#define PROX7_REFLECTED 15
#define GROUND0_AMBIENT 16
#define GROUND0_REFLECTED 17
#define GROUND1_AMBIENT 18
#define GROUND1_REFLECTED 19
#define GROUND2_AMBIENT 20
#define GROUND2_REFLECTED 21
#define GROUND3_AMBIENT 22
#define GROUND3_REFLECTED 23
#endif

/***********************/
/*** CLIFF AVOIDANCE ***/
/***********************/
#ifndef CLIFF_THR
#define CLIFF_THR 420						// ground sensors threshold for detecting a cliff independently of the surface
#endif										// since the ground sensors are calibrated to 512

/**************************/
/*** OBSTACLE AVOIDANCE ***/
/**************************/
#ifndef PHASE1								// range values used for linearization of the proximity; this values define 
#define PHASE1 60							// the passage from one to the next linear function used to approximate and 
#endif										// linearize the proximity values
#ifndef PHASE2
#define PHASE2 120
#endif
#ifndef PHASE3
#define PHASE3 180
#endif
#ifndef NOISE_THR							// define the value under which the proximity is considered to be noise
#define NOISE_THR 5
#endif

/****************/
/*** ODOMETRY ***/
/****************/
#define WHEEL_DIST 39.5 		// mm
// the encoders values depends on desired speed: enc = enc + meas_speed*k, where k=function(desired_speed)= a*desired_speed + b
// the following values represent the "b" (offset) and "a" (slope) of the function 
#define LEFT_ENC_OFFSET 80.8	// these values are obtained from field test
#define RIGHT_ENC_OFFSET 78.6
#define ENC_SLOPE 0.384

#define LEFT_WHEEL_FW_SC 0                  // forward, speed control enabled
#define RIGHT_WHEEL_FW_SC 1
#define LEFT_WHEEL_BW_SC 2					// backward, speed control enabled
#define RIGHT_WHEEL_BW_SC 3
#define DISTANCE_MM 248.18					// distance traveled during calibration => WHEEL_DIST*2*PI
#define INDEX_STEP 5 //14
#define BYTE_TO_MM_S 5
#define CALIBRATION_SAMPLES 9
#define CALIB_CHECK_ADDRESS 3946
#define CALIB_DATA_START_ADDR 3948

#define CALIBRATION_STATE_SET_SPEED 0
#define CALIBRATION_STATE_START_MEASURE 1
#define CALIBRATION_STATE_EXIT_BLACK_LINE_1 2
#define CALIBRATION_STATE_STOP_MEASURE 3
#define CALIBRATION_STATE_EXIT_BLACK_LINE_2 4
#define CALIBRATION_STATE_NEXT 5
#define CALIBRATION_STATE_FIND_THRS_0 6
#define CALIBRATION_STATE_FIND_THRS_1 7
#define CALIBRATION_STATE_FIND_THRS_2 8



/***************/
/*** IR COMM ***/
/***************/
#define IRCOMM_MODE_SENSORS_SAMPLING 0
#define IRCOMM_MODE_RECEIVE 1
#define IRCOMM_MODE_TRANSMIT 2
//#define IRCOMM_MODE_RECEIVE_ONLY 3
//#define IRCOMM_MODE_TRANSMIT_ONLY 4
#define IRCOMM_SAMPLING_WINDOW 20


// reception
#define IRCOMM_RX_IDLE_STATE 1
#define IRCOMM_RX_MAX_SENSOR_STATE 2 
#define IRCOMM_RX_DETECT_START_BIT_STATE 3
#define IRCOMM_RX_SYNC_SIGNAL 4
#define IRCOMM_RX_WAITING_BIT 5
#define IRCOMM_RX_READ_BIT 6
#define IRCOMM_RX_CHECK_CRC 7
#define IRCOMM_RX_DEBUG 8
#define IRCOMM_RX_STOP 9

#define IRCOMM_DETECTION_AMPLITUDE_THR 80 //20
#define IRCOMM_START_BIT_MIN_SWITCH_COUNT 2

// transmission
#define IRCOMM_TX_IDLE_STATE 1				// wait the user data to be sent
#define IRCOMM_TX_COMPUTE_TIMINGS 2 
#define IRCOMM_TX_TRANSMIT_BIT 3
#define IRCOMM_TX_PREPARE_TRANSMISSION 5

#define IRCOMM_TX_ADC_IDLE 0
#define IRCOMM_TX_ADC_TURN_OFF_SENSORS 1
#define IRCOMM_TX_ADC_WAIT_PREPARATION 2
#define IRCOMM_TX_ADC_TRANSMISSION_SEQ1 3	// we need two state for byte transmission to change the channel sampled
#define IRCOMM_TX_ADC_TRANSMISSION_SEQ2 4	// related to motors

#define IRCOMM_BIT_START1_SWITCH_COUNT 2
#define IRCOMM_BIT_START2_SWITCH_COUNT 1
#define IRCOMM_BIT0_SWITCH_COUNT 8
#define IRCOMM_BIT1_SWITCH_COUNT 4
#define IRCOMM_BIT_START1_DURATION 120
#define IRCOMM_BIT_START2_DURATION 240
#define IRCOMM_BIT0_DURATION 30	// based on adc isr of 104 us
#define IRCOMM_BIT1_DURATOIN 60 // based on adc isr of 104 us

// debug
#define DEBUG_MAX_SENSOR_STATE 0
#define DEBUG_ALL_SENSORS 0
#define DEBUG_MAX_SENSOR 0
#define DEBUG_START_BIT_STATE 0
#define DEBUG_READ_BIT 0
#define DEBUG_BYTE_RECEPTION 0
#define DEBUG_BYTE_SEQUENCE 0


