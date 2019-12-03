#include "constants.h"

/***********/
/*** ADC ***/
/***********/
extern volatile unsigned char currentAdChannel;
extern unsigned char currentProx;
extern unsigned char currentMotLeftChannel;
extern unsigned char currentMotRightChannel;
extern unsigned char rightMotorPhase;
extern unsigned char leftMotorPhase;
extern volatile unsigned int proximityValue[24];
extern int proximityResult[12];
extern signed int proximityOffset[12];
extern unsigned long proximitySum[12];
extern unsigned char adcSaveDataTo;
extern unsigned char adcSamplingState;
extern unsigned char rightChannelPhase;
extern unsigned char leftChannelPhase;
extern unsigned int batteryLevel;
extern unsigned char measBattery;
extern unsigned char proxUpdated;
extern int proximityResultLinear[8];
extern float rightMotSteps;
extern float leftMotSteps;
extern float rightMotStepsOld;
extern float leftMotStepsOld;

/******************************/
/*** CONSUMPTION CONTROLLER ***/
/******************************/
extern unsigned int left_current_avg;
extern unsigned int right_current_avg;
extern unsigned int last_left_current;
extern unsigned int last_right_current;

/************************/
/*** SPEED CONTROLLER ***/
/************************/
extern signed int pwm_right_desired_to_control;
extern signed int pwm_left_desired_to_control;
extern unsigned int left_vel_sum;
extern unsigned int right_vel_sum;
extern signed int last_left_vel;
extern signed int last_right_vel;
extern signed int pwm_right;
extern signed int pwm_left;
extern signed int pwm_right_desired;
extern signed int pwm_left_desired;
extern signed int pwm_intermediate_right_desired;
extern signed int pwm_intermediate_left_desired;
extern signed long int pwm_right_speed_controller;
extern signed long int pwm_left_speed_controller;
extern signed int delta_left_speed_current;
extern signed int delta_right_speed_current;
extern signed int delta_left_speed_prev;
extern signed int delta_right_speed_prev;
extern signed int delta_left_speed_sum;
extern signed int delta_right_speed_sum;
extern unsigned char compute_left_vel;
extern unsigned char compute_right_vel;
extern signed int pwm_right_working;
extern signed int pwm_left_working;
extern unsigned char firstSampleRight;
extern unsigned char firstSampleLeft;

/***********/
/*** NRF ***/
/***********/
extern unsigned int dataLED[3];
extern signed int speedl;
extern signed int speedr;
extern unsigned char rfData[PAYLOAD_SIZE];
extern unsigned char ackPayload[16];
extern unsigned char packetId;
extern unsigned int rfAddress;
extern unsigned char rfFlags;
extern unsigned char spiCommError;
extern unsigned char rfDebugMode;
extern unsigned char rfDebugCounter;

/****************/
/*** RGB LEDS ***/
/****************/
extern unsigned char pwm_red;
extern unsigned char pwm_green;
extern unsigned char pwm_blue;
extern unsigned char blinkState;
extern unsigned char rgbState;

/************/
/*** UART ***/
/************/
extern unsigned char peripheralChoice;
extern unsigned char choosePeripheral;
extern unsigned char sendAdcValues;
extern unsigned char commError;
extern unsigned int byteCount;
extern unsigned char uartBuff[UART_BUFF_SIZE];
extern unsigned char nextByteIndex;
extern unsigned char currByteIndex;
extern unsigned char chooseMenu;
extern unsigned char menuChoice;
extern unsigned char addressReceived;
extern unsigned char menuState;
extern unsigned char getDataNow;

/*************************/
/*** IR REMOTE CONTROL ***/
/*************************/
extern unsigned char irCommand;
extern unsigned char command_received;
extern unsigned char colorState;
extern unsigned char irEnabled;
extern unsigned char checkGlitch;
extern unsigned char behaviorState;
extern uint32_t lastCmdTime;

/*********************/
/*** ACCELEROMETER ***/
/*********************/
extern unsigned char accelAddress;
extern unsigned char useAccel;
extern signed int accX;
extern signed int accY;
extern signed int accZ;
extern signed int accOffsetX;
extern signed int accOffsetY;
extern signed int accOffsetXSum;
extern signed int accOffsetYSum;
extern signed int accXMax, accXMin, accYMax, accYMin;
extern signed int currentAngle;
extern unsigned char currPosition;
extern unsigned int timesInSamePos;
extern unsigned char robotPosition;
extern signed char accBuff[6];
extern unsigned temperature;

/***************/
/*** VARIOUS ***/
/***************/
extern uint32_t clockTick;
extern unsigned char currentSelector;
extern signed int calibrationCycle;
extern unsigned char startCalibration;
extern unsigned char hardwareRevision;
extern unsigned char demoState;
extern unsigned char lineFound;
extern unsigned char outOfLine;
extern unsigned char chargeContact;
extern unsigned long int demoStartTime;
extern unsigned long int demoStartTime2;
extern unsigned long int demoEndTime;
extern unsigned char currentOsccal;
extern unsigned long long int speedStepCounter;
extern unsigned char speedStep;
extern unsigned char softAccEnabled;
extern unsigned char calibrationWritten;
extern unsigned char greenLedState;
extern unsigned char rgbLedState;
extern uint32_t lastTick;

/**************************/
/*** OBSTACLE AVOIDANCE ***/
/**************************/
extern unsigned char obstacleAvoidanceEnabled;

/***********************/
/*** CLIFF AVOIDANCE ***/
/***********************/
extern unsigned char cliffAvoidanceEnabled;
extern unsigned char cliffDetectedFlag;

/****************/
/*** ODOMETRY ***/
/****************/
extern float theta, lastTheta, xPos, yPos, deltaDist;
extern float thetaOld, xPosOld, yPosOld, deltaDistOld;
extern float leftDist, rightDist, leftDistPrev, rightDistPrev;
extern unsigned char computeOdometry;
extern float thetaAcc;
extern unsigned char calibState;
extern unsigned char calibVelIndex;
extern unsigned char calibWheel;
extern signed int tempVel;
extern signed int calibration[CALIBRATION_SAMPLES][8];
extern unsigned long int timeoutOdometry;
extern unsigned long timeOdometry;
extern unsigned char calibrateOdomFlag;
extern signed long int leftSpeedSumOdom;
extern signed long int rightSpeedSumOdom;
extern unsigned int leftSumCount;
extern unsigned int rightSumCount;
extern signed int avgLeftSpeed;
extern signed int avgRightSpeed;
extern signed int speedLeftFromEnc;
extern signed int speedRightFromEnc;
extern uint32_t timeLeftOdom;
extern uint32_t timeRightOdom;
extern int minGround;
extern int maxGround;
extern int calibrationThr;
//extern int calibrationThrLow;
//extern int calibrationThrHigh;

/***************/
/*** IR COMM ***/
/***************/
extern unsigned char irCommEnabled;
extern unsigned char irCommEnabledNext;
extern unsigned char irCommMode;
extern volatile unsigned char irCommState;
extern unsigned int irCommTempValue;
extern volatile unsigned char irCommSendValues;
extern unsigned long int irCommTickCounter;
extern unsigned long int irCommTickCounter2;
extern unsigned char irCommTickCounterUpdate;
extern signed char irCommLastSensor;
extern signed int irCommLastData;

// demo
extern signed int angleDeg;
extern signed int angleError;
extern unsigned char angleDegEncode;
extern unsigned char irCommRobotId;
extern unsigned char irCommRobotsNum;
extern unsigned char irCommLedToggle;
extern unsigned char irCommMsgCount;

// debug
extern signed int irCommMaxSensorSignalTemp[IRCOMM_SAMPLING_WINDOW*2];
extern unsigned char irCommMaxSensorSignalIndexTemp;
extern signed int irCommMaxSensorSignalFiltTemp[IRCOMM_SAMPLING_WINDOW*2];
extern unsigned char irCommMaxSensorSignalFiltIndexTemp;
extern signed int irCommRxMaxSensorTemp[2];
extern unsigned char irCommRxMaxSensorIndexTemp;
extern signed int irCommRxMaxDiffTemp[2];
extern unsigned char irCommRxMaxDiffIndexTemp;
extern signed int irCommProxMeanTemp[2];
extern unsigned char irCommProxMeanIndexTemp;
extern unsigned char irCommSwitchCountTemp[2];
extern unsigned char irCommSwitchCountIndexTemp;
extern unsigned int irCommMaxSensorValueCurrTemp[2];
extern unsigned char irCommMaxSensorValueCurrIndexTemp;
extern unsigned int irCommMinSensorValueCurrTemp[2];
extern unsigned char irCommMinSensorValueCurrIndexTemp;
extern unsigned char irCommShiftCountTemp[2];
extern unsigned char irCommShiftCountIndexTemp;
extern unsigned char irCommShiftCountFinalTemp[2];
extern unsigned char irCommShiftCountFinalIndexTemp;
extern unsigned int irCommStartDiffTemp[2];
extern unsigned char irCommStartDiffIndexTemp;
extern unsigned char irCommComputeShiftTemp[2];
extern unsigned char irCommComputeShiftIndexTemp;
extern unsigned char irCommRxPeakHighToLowTemp[2];
extern unsigned char irCommRxPeakHighToLowIndexTemp;
extern unsigned char irCommRxStartPeakDurationTemp[2];
extern unsigned char irCommRxStartPeakDurationIndexTemp;
extern unsigned char irCommRxStartBitDetectedTemp[2];
extern unsigned char irCommRxStartBitDetectedIndexTemp;
extern unsigned char irCommStateTemp[14];
extern unsigned char irCommStateIndexTemp;
extern unsigned char irCommSyncStateTemp[2];
extern unsigned char irCommSyncStateIndexTemp;
extern signed int irCommBitsSignalTemp[IRCOMM_SAMPLING_WINDOW*10];
extern unsigned char irCommBitsSignalIndexTemp;
extern unsigned char irCommRxBitReceivedTemp[10];
extern unsigned char irCommRxBitReceivedIndexTemp;
extern unsigned char irCommRxPrevDataReceived;

// reception
extern unsigned char irCommAdcRxState;
extern unsigned char irCommRxWindowSamples;
extern unsigned int irCommMaxSensorValueBuff1[8];
extern unsigned int irCommMaxSensorValueBuff2[8];
extern unsigned int *irCommMaxSensorValueAdc;
extern unsigned int *irCommMaxSensorValueCurr;
extern unsigned int irCommMinSensorValueBuff1[8];
extern unsigned int irCommMinSensorValueBuff2[8];
extern unsigned int *irCommMinSensorValueAdc;
extern unsigned int *irCommMinSensorValueCurr;
extern unsigned int irCommProxValuesBuff1[8*IRCOMM_SAMPLING_WINDOW];
extern unsigned int irCommProxValuesBuff2[8*IRCOMM_SAMPLING_WINDOW];
extern unsigned int *irCommProxValuesAdc;
extern unsigned int *irCommProxValuesCurr;
extern unsigned int *irCommTempPointer;
extern unsigned char irCommRxCrc;
extern signed int irCommMaxSensorSignal[IRCOMM_SAMPLING_WINDOW];
extern signed long int irCommProxSum;
extern signed int irCommTempMax;
extern signed int irCommTempMin;
extern unsigned char irCommShiftCount;
extern unsigned char irCommComputeShift;
extern signed int irCommProxMean;
extern signed char irCommSignalState;
extern unsigned char irCommSwitchCount;
extern unsigned char irCommRxBitCount;
extern unsigned char irCommRxCrcError;
extern unsigned char irCommRxByte;
extern unsigned char irCommSecondBitSkipped;
extern unsigned char irCommShiftCounter;
extern unsigned char irCommRxBitReceived[10];
extern unsigned char irCommRxByteExpected;
extern unsigned char irCommRxSequenceCount;
extern unsigned char irCommRxLastDataReceived;
extern unsigned char irCommRxDataAvailable;
extern signed char irCommRxReceivingSensor;
extern unsigned char irCommRxBitSkipped;
extern unsigned char irCommRxStartBitDetected;
extern unsigned char irCommRxPeakHighToLow;
extern unsigned char irCommRxStartPeakDuration;
extern signed int irCommRxMaxDiff;
extern signed int irCommRxMaxSensor;
extern unsigned char irCommRxNumReceivingSensors;

// transmission
extern unsigned char irCommAdcTxState;
extern unsigned char irCommTxByte;
extern unsigned char irCommTxByteEnqueued;
extern unsigned long int irCommTxLastTransmissionTime;
extern unsigned char irCommTxBitToTransmit[12];
extern unsigned char irCommTxCrc;
extern unsigned char irCommTxBitCount;
extern unsigned char irCommTxPulseState;
extern unsigned int irCommTxDuration;
extern unsigned char irCommTxSwitchCount;
extern unsigned char irCommTxSwitchCounter;
extern unsigned char irCommTxDurationCycle;
extern unsigned char irCommTxSensorMask;
extern unsigned char irCommTxSensorGroup;









