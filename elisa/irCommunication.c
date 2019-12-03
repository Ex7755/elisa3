#include "irCommunication.h"


void irCommInitTransmitter() {
	irCommEnabled = IRCOMM_MODE_TRANSMIT;
	irCommState = IRCOMM_TX_IDLE_STATE;
}

void irCommInitReceiver() {
	irCommEnabled = IRCOMM_MODE_RECEIVE;
	irCommState = IRCOMM_RX_IDLE_STATE;
	irCommEnabledNext = IRCOMM_MODE_RECEIVE;
}

void irCommInitVars() {
	memset(irCommMaxSensorValueAdc, 0x00, 16);
	memset(irCommMinSensorValueAdc, 0xFF, 16);
	irCommMode = IRCOMM_MODE_SENSORS_SAMPLING;
}

void irCommInit() {
	irCommProxValuesAdc = irCommProxValuesBuff1;
	irCommProxValuesCurr = irCommProxValuesBuff2;
	irCommMaxSensorValueAdc = irCommMaxSensorValueBuff1;
	irCommMaxSensorValueCurr = irCommMaxSensorValueBuff2;
	irCommMinSensorValueAdc = irCommMinSensorValueBuff1;
	irCommMinSensorValueCurr = irCommMinSensorValueBuff2;
	irCommInitVars();
	irCommInitReceiver();
}

void irCommDeinit() {
	irCommEnabled = IRCOMM_MODE_SENSORS_SAMPLING;
	irCommMode = IRCOMM_MODE_SENSORS_SAMPLING;
}

void resetDebugVariables() {
	irCommRxMaxSensorIndexTemp=0;
	irCommRxMaxDiffIndexTemp=0;
	irCommMaxSensorValueCurrIndexTemp=0;
	irCommMinSensorValueCurrIndexTemp=0;
	memset(irCommRxMaxSensorTemp, 0xFF, 4);
	memset(irCommRxMaxDiffTemp, 0xFF, 4);
	memset(irCommMaxSensorValueCurrTemp, 0xFF, 4);
	memset(irCommMinSensorValueCurrTemp, 0xFF, 4);

	irCommStateIndexTemp = 0;					
	memset(irCommStateTemp, 0xFF, 14);

	irCommShiftCountFinalIndexTemp = 0;
	memset(irCommShiftCountFinalTemp, 0xFF, 2);
	irCommRxStartBitDetectedIndexTemp = 0;
	memset(irCommRxStartBitDetectedTemp, 0xFF, 2);
	irCommSwitchCountIndexTemp = 0;
	memset(irCommSwitchCountTemp, 0xFF, 2);
	irCommMaxSensorSignalFiltIndexTemp = 0;
	memset(irCommMaxSensorSignalFiltTemp, 0xFF, 80);
	irCommMaxSensorSignalIndexTemp = 0;
	memset(irCommMaxSensorSignalTemp, 0xFF, 80);
	irCommProxMeanIndexTemp = 0;
	memset(irCommProxMeanTemp, 0xFF, 4);
	irCommComputeShiftIndexTemp = 0;
	memset(irCommComputeShiftTemp, 0xFF, 2);
	irCommShiftCountIndexTemp = 0;
	memset(irCommShiftCountTemp, 0xFF, 2);
	irCommRxPeakHighToLowIndexTemp = 0;
	memset(irCommRxPeakHighToLowTemp, 0xFF, 2);
	irCommRxStartPeakDurationIndexTemp = 0;
	memset(irCommRxStartPeakDurationTemp, 0xFF, 2);
	irCommStartDiffIndexTemp = 0;
	memset(irCommSyncStateTemp, 0xFF, 2);
	irCommSyncStateIndexTemp = 0;
	memset(irCommBitsSignalTemp, 0xFF, 400);
	irCommBitsSignalIndexTemp = 0;
	memset(irCommRxBitReceivedTemp, 0xFF, 10);
	irCommRxBitReceivedIndexTemp = 0;
	
	
}

void irCommTasks() {
	int i = 0;

	if(irCommMode==IRCOMM_MODE_RECEIVE) {

		switch(irCommState) {
			case IRCOMM_RX_IDLE_STATE:				
				if((irCommRxStartBitDetected==0) && (irCommEnabled!=irCommEnabledNext)) {
					if((getTime100MicroSec() - irCommTxLastTransmissionTime) > PAUSE_200_MSEC) {
						irCommInitTransmitter();
					}					
				}
				break;

			case IRCOMM_RX_MAX_SENSOR_STATE:
				// check from how many sensors the robot is receiving a possible message
				for(i=0; i<8; i++) {
					if((signed int)(irCommMaxSensorValueCurr[i]-irCommMinSensorValueCurr[i]) >= IRCOMM_DETECTION_AMPLITUDE_THR) {
						irCommRxNumReceivingSensors++;
					}					
				}
				if(irCommRxNumReceivingSensors==0) {
					//if(irCommRxStartBitDetected == 1) {	// signal becomes too low to be reliable...or something else happened				
					//	irCommState = IRCOMM_RX_DEBUG;
					//	irCommAdcRxState = 12;
					//	updateGreenLed(0);
					//} else {
						irCommRxStartBitDetected = 0;
						currentProx = 0;
						adcSaveDataTo = SKIP_SAMPLE;
						adcSamplingState = 0;
						irCommMode=IRCOMM_MODE_SENSORS_SAMPLING;
						irCommState = IRCOMM_RX_IDLE_STATE;
						resetDebugVariables();
					//}			
					// start listening from the next sensor the next time I check for a start bit in order to get the same chance 
					// to all sensors, instead of always listening from the sensor that has the best signal
					if(irCommRxMaxSensor < 7) {
						irCommRxMaxSensor++;
					} else {
						irCommRxMaxSensor = 0;
					}
				} else {
					if(irCommRxStartBitDetected==0) {
						// start listening from the next sensor
						if(irCommRxMaxSensor < 7) {
							irCommRxMaxSensor++;
						} else {
							irCommRxMaxSensor = 0;
						}
						// check which is the next sensor that has a reliable signal
						i = 0;
						while(i<8) {
							if((signed int)(irCommMaxSensorValueCurr[irCommRxMaxSensor]-irCommMinSensorValueCurr[irCommRxMaxSensor]) >= IRCOMM_DETECTION_AMPLITUDE_THR) {
								irCommRxMaxDiff = irCommMaxSensorValueCurr[irCommRxMaxSensor]-irCommMinSensorValueCurr[irCommRxMaxSensor];
								break;
							}
							if(irCommRxMaxSensor < 7) {
								irCommRxMaxSensor++;
							} else {
								irCommRxMaxSensor = 0;
							}
							i++;
						}
					} else {	// listen from the same sensor when a start bit is detected
						irCommRxMaxDiff = irCommMaxSensorValueCurr[irCommRxMaxSensor]-irCommMinSensorValueCurr[irCommRxMaxSensor];
					}
				}
				//turnOffGreenLeds();
				//setGreenLed(irCommRxMaxSensor, 1);			

				if(irCommRxMaxSensorIndexTemp>1) {
					irCommRxMaxSensorIndexTemp = 1;
					//updateBlueLed(0);
				}
				irCommRxMaxSensorTemp[irCommRxMaxSensorIndexTemp] = irCommRxMaxSensor;
				irCommRxMaxSensorIndexTemp++;
					
				if(irCommRxMaxDiffIndexTemp>1) {
					irCommRxMaxDiffIndexTemp = 1;
					//updateBlueLed(0);
				}
				irCommRxMaxDiffTemp[irCommRxMaxDiffIndexTemp] = irCommRxMaxDiff;
				irCommRxMaxDiffIndexTemp++;
				
				if(irCommMaxSensorValueCurrIndexTemp>1) {
					irCommMaxSensorValueCurrIndexTemp = 1;
					//updateBlueLed(0);
				}
				irCommMaxSensorValueCurrTemp[irCommMaxSensorValueCurrIndexTemp] = irCommMaxSensorValueCurr[irCommRxMaxSensor];
				irCommMaxSensorValueCurrIndexTemp++;
				
				if(irCommMinSensorValueCurrIndexTemp>1) {
					irCommMinSensorValueCurrIndexTemp = 1;
					//updateBlueLed(0);
				}
				irCommMinSensorValueCurrTemp[irCommMinSensorValueCurrIndexTemp] = irCommMinSensorValueCurr[irCommRxMaxSensor];
				irCommMinSensorValueCurrIndexTemp++;			


				//if(irCommRxMaxSensor == -1) {
				//	updateRedLed(0);
				//}
				if(irCommRxMaxDiff >= IRCOMM_DETECTION_AMPLITUDE_THR) {
					irCommState = IRCOMM_RX_DETECT_START_BIT_STATE;	
					
					if(irCommStateIndexTemp>13) {
						irCommStateIndexTemp = 13;
						//updateBlueLed(0);
					}
					irCommStateTemp[irCommStateIndexTemp] = irCommState;
					irCommStateIndexTemp++;
					

					// transmit debug information
					if(DEBUG_MAX_SENSOR_STATE) {
						irCommSendValues = 0;						
						while(irCommSendValues==0);	// wait for the start from the uart (computer)
						usart0Transmit(0xFF, 1);
						usart0Transmit(irCommRxMaxSensor,1);
						usart0Transmit(irCommRxMaxDiff&0xFF,1);
						usart0Transmit(irCommRxMaxDiff>>8,1);
						if(DEBUG_ALL_SENSORS) {
							for(i=0; i<8*IRCOMM_SAMPLING_WINDOW; i++) {
								usart0Transmit(irCommProxValuesCurr[i]&0xFF,1);
								usart0Transmit(irCommProxValuesCurr[i]>>8,1);
							}
						} else if(DEBUG_MAX_SENSOR) {
							for(i=0; i<IRCOMM_SAMPLING_WINDOW; i++) {
								irCommTempValue = irCommProxValuesCurr[irCommRxMaxSensor+i*8];
								usart0Transmit(irCommTempValue&0xFF,1);
								usart0Transmit(irCommTempValue>>8,1);
							}
						}
					}
				} else {
					// cannot get a reliable signal from the sensor from which the start bit was detected the previous time,
					// thus restart listening from the next sensor
					if(irCommRxStartBitDetected == 1) {
						if(irCommRxMaxSensor < 7) {
							irCommRxMaxSensor++;
						} else {
							irCommRxMaxSensor = 0;
						}	
					}

					//if(irCommRxStartBitDetected == 1) {	// signal becomes too low to be reliable...or something else happened				
					//	irCommState = IRCOMM_RX_DEBUG;
					//	irCommAdcRxState = 12;
					//	updateGreenLed(0);
					//} else {
						irCommRxStartBitDetected = 0;
						currentProx = 0;
						adcSaveDataTo = SKIP_SAMPLE;
						adcSamplingState = 0;
						irCommMode=IRCOMM_MODE_SENSORS_SAMPLING;
						irCommState = IRCOMM_RX_IDLE_STATE;
						resetDebugVariables();
													
				}	

												
				break;

			case IRCOMM_RX_DETECT_START_BIT_STATE:
				// extract signal from the sensor with higher amplitude and compute the signal mean
				irCommProxSum = 0;
				irCommTempMin = 1024;
				irCommTempMax = 0;
				irCommShiftCount = 0;
				irCommComputeShift = 1;												
				for(i=0; i<IRCOMM_SAMPLING_WINDOW; i++) {
					irCommMaxSensorSignal[i] = irCommProxValuesCurr[irCommRxMaxSensor+i*8];

					if(irCommMaxSensorSignalIndexTemp>39) {
						irCommMaxSensorSignalIndexTemp = 39;
						//updateRedLed(0);
					}
					irCommMaxSensorSignalTemp[irCommMaxSensorSignalIndexTemp] = irCommMaxSensorSignal[i];
					irCommMaxSensorSignalIndexTemp++;					

					irCommProxSum += irCommMaxSensorSignal[i];
					if(irCommComputeShift == 1) {	// compute the shift between the rx sampling and tx signal
						irCommShiftCount++;								
						if(irCommTempMin > irCommMaxSensorSignal[i]) {
							irCommTempMin = irCommMaxSensorSignal[i];
						}
						if(irCommTempMax < irCommMaxSensorSignal[i]) {
							irCommTempMax = irCommMaxSensorSignal[i];
						}
						irCommTempValue = (irCommTempMax - irCommTempMin);						
						if((irCommTempMax - irCommTempMin) >= IRCOMM_DETECTION_AMPLITUDE_THR) {

							if(irCommStartDiffIndexTemp>1) {
								irCommStartDiffIndexTemp = 1;
								//updateRedLed(0);
							}
							irCommStartDiffTemp[irCommStartDiffIndexTemp] = irCommTempValue;
							irCommStartDiffIndexTemp++;
							

							if(irCommMaxSensorSignal[i] == irCommTempMax) {
								irCommRxPeakHighToLow = 0;
							} else if(irCommMaxSensorSignal[i] == irCommTempMin) {
								irCommRxPeakHighToLow = 1;
							} else {
								//updateGreenLed(0);
							}
							irCommTempMin = irCommMaxSensorSignal[i];	// otherwise I cannot detect spike of one sample (peakDuration=1), related to sensors sampling
							irCommTempMax = irCommMaxSensorSignal[i];
							irCommComputeShift = 2;
							irCommShiftCount--;	// the current sample is already part of the signal start thus do not skip it
							irCommRxStartPeakDuration = 0;
						}
					 } else if(irCommComputeShift == 2) {		
					 	irCommRxStartPeakDuration++;					
						if(irCommTempMin > irCommMaxSensorSignal[i]) {
							irCommTempMin = irCommMaxSensorSignal[i];
						}
						if(irCommTempMax < irCommMaxSensorSignal[i]) {
							irCommTempMax = irCommMaxSensorSignal[i];
						}	
						if((irCommTempMax - irCommTempMin) >= IRCOMM_DETECTION_AMPLITUDE_THR) {
							if((irCommMaxSensorSignal[i]==irCommTempMax) && (irCommRxPeakHighToLow==1)) {
								irCommComputeShift = 0;
							} else if((irCommMaxSensorSignal[i]==irCommTempMin) && (irCommRxPeakHighToLow==0)) {
								irCommComputeShift = 0;
							} else {
								//updateGreenLed(0);
							}
							
						}											
					 }

				}

				if(irCommComputeShiftIndexTemp>1) {
					irCommComputeShiftIndexTemp = 1;
					//updateRedLed(0);
				}
				irCommComputeShiftTemp[irCommComputeShiftIndexTemp] = irCommComputeShift;
				irCommComputeShiftIndexTemp++;
				
				if(irCommShiftCountIndexTemp>1) {
					irCommShiftCountIndexTemp = 1;
					//updateRedLed(0);
				}
				irCommShiftCountTemp[irCommShiftCountIndexTemp] = irCommShiftCount;
				irCommShiftCountIndexTemp++;
				
				if(irCommRxPeakHighToLowIndexTemp>1) {
					irCommRxPeakHighToLowIndexTemp = 1;
					//updateRedLed(0);
				}
				irCommRxPeakHighToLowTemp[irCommRxPeakHighToLowIndexTemp] = irCommRxPeakHighToLow;
				irCommRxPeakHighToLowIndexTemp++;
				
				if(irCommRxStartPeakDurationIndexTemp>1) {
					irCommRxStartPeakDurationIndexTemp = 1;
					//updateRedLed(0);
				}
				irCommRxStartPeakDurationTemp[irCommRxStartPeakDurationIndexTemp] = irCommRxStartPeakDuration;
				irCommRxStartPeakDurationIndexTemp++;
					
				
				//if(irCommComputeShift != 0) {	// it should not be never 1 because the difference between min and max in the current signal
					//updateRedLed(0);			// is at least IRCOMM_DETECTION_AMPLITUDE_THR (checked in the previous state)
				//}
				irCommProxMean = (int)(irCommProxSum / IRCOMM_SAMPLING_WINDOW);

				if(irCommProxMeanIndexTemp>1) {
					irCommProxMeanIndexTemp = 1;
					//updateRedLed(0);
				}
				irCommProxMeanTemp[irCommProxMeanIndexTemp] = irCommProxMean;
				irCommProxMeanIndexTemp++;
				

				// substract mean from signal
				for(i=0; i<IRCOMM_SAMPLING_WINDOW; i++) {
					irCommMaxSensorSignal[i] -= irCommProxMean;

					if(irCommMaxSensorSignalFiltIndexTemp>39) {
						irCommMaxSensorSignalFiltIndexTemp = 39;
						//updateRedLed(0);
					}
					irCommMaxSensorSignalFiltTemp[irCommMaxSensorSignalFiltIndexTemp] = irCommMaxSensorSignal[i];
					irCommMaxSensorSignalFiltIndexTemp++;
					
				}
						
				// start counting number of switch around mean signal
				if(irCommMaxSensorSignal[0] > 0) {
					irCommSignalState = 1;
				} else {
					irCommSignalState = -1;
				}
				irCommSwitchCount = 0;
				for(i=1; i<IRCOMM_SAMPLING_WINDOW; i++) {
					if(irCommMaxSensorSignal[i] > 0) {
						if(irCommSignalState < 0) {
							irCommSignalState = 1;
							irCommSwitchCount++;
						}
					} else {
						if(irCommSignalState > 0) {
							irCommSignalState = -1;
							irCommSwitchCount++;
						}
					}
				}

				if(irCommSwitchCountIndexTemp>1) {
					irCommSwitchCountIndexTemp = 1;
					//updateRedLed(0);
				}
				irCommSwitchCountTemp[irCommSwitchCountIndexTemp] = irCommSwitchCount;
				irCommSwitchCountIndexTemp++;
				

				//turnOffGreenLeds();
				if(irCommRxPeakHighToLow==1) {
					if(irCommRxStartBitDetected==1) {
						if(irCommSwitchCount==2) {
							if(irCommSyncStateIndexTemp>1) {
								irCommSyncStateIndexTemp = 1;
								//updateRedLed(0);
							}
							irCommSyncStateTemp[irCommSyncStateIndexTemp] = 6;
							irCommSyncStateIndexTemp++;

							irCommShiftCount = 0;
							irCommRxPeakHighToLow = 0;
							irCommRxStartBitDetected = 0;
							irCommSecondBitSkipped = 0;
							irCommShiftCounter = 0;
							irCommRxBitCount = 0;	
							irCommRxCrc = 0;	
							irCommRxByte = 0;
							irCommState = IRCOMM_RX_WAITING_BIT;
						} else if(irCommSwitchCount==1) {
							if(irCommSyncStateIndexTemp>1) {
								irCommSyncStateIndexTemp = 1;
								//updateRedLed(0);
							}
							irCommSyncStateTemp[irCommSyncStateIndexTemp] = 2;
							irCommSyncStateIndexTemp++;

							//irCommShiftCount = irCommShiftCount;
							irCommRxPeakHighToLow = 0;
							irCommRxStartBitDetected = 0;
							irCommSecondBitSkipped = 0;
							irCommShiftCounter = 0;
							irCommRxBitCount = 0;	
							irCommRxCrc = 0;	
							irCommRxByte = 0;
							irCommState = IRCOMM_RX_SYNC_SIGNAL;
						} else {
							irCommRxStartBitDetected = 0;
							currentProx = 0;
							adcSaveDataTo = SKIP_SAMPLE;
							adcSamplingState = 0;
							irCommMode=IRCOMM_MODE_SENSORS_SAMPLING;
							irCommState = IRCOMM_RX_IDLE_STATE;	

							if(irCommSyncStateIndexTemp>1) {
								irCommSyncStateIndexTemp = 1;
								//updateRedLed(0);
							}
							irCommSyncStateTemp[irCommSyncStateIndexTemp] = 12;
							irCommSyncStateIndexTemp++;
							
							//irCommState = IRCOMM_RX_DEBUG;
							//irCommAdcRxState = 12;
							//updateRedLed(0);
							//break;

							resetDebugVariables();				
							break;
						}
					} else {
						if(irCommSwitchCount==2) {
							if(irCommRxStartPeakDuration<=3) {	// peak due to sensors sampling detected
								currentProx = 0;
								adcSaveDataTo = SKIP_SAMPLE;
								adcSamplingState = 0;
								irCommMode=IRCOMM_MODE_SENSORS_SAMPLING;							
								irCommState = IRCOMM_RX_IDLE_STATE;

								if(irCommSyncStateIndexTemp>1) {
									irCommSyncStateIndexTemp = 1;
									//updateRedLed(0);
								}
								irCommSyncStateTemp[irCommSyncStateIndexTemp] = 16;
								irCommSyncStateIndexTemp++;
							
								//irCommState = IRCOMM_RX_DEBUG;
								//irCommAdcRxState = 12;
								//updateGreenLed(0);
								//break;

								resetDebugVariables();
								break;
							} else {
								if(irCommSyncStateIndexTemp>1) {
									irCommSyncStateIndexTemp = 1;
									//updateRedLed(0);
								}
								irCommSyncStateTemp[irCommSyncStateIndexTemp] = 3;
								irCommSyncStateIndexTemp++;

								irCommShiftCount = IRCOMM_SAMPLING_WINDOW + irCommShiftCount;
								irCommRxPeakHighToLow = 0;
								irCommRxStartBitDetected = 0;
								irCommSecondBitSkipped = 0;
								irCommShiftCounter = 0;
								irCommRxBitCount = 0;	
								irCommRxCrc = 0;	
								irCommRxByte = 0;
								irCommState = IRCOMM_RX_SYNC_SIGNAL;
							}
						} else if(irCommSwitchCount==1) {		
							irCommRxStartBitDetected = 1;					
							if(irCommRxStartPeakDuration > IRCOMM_SAMPLING_WINDOW/2) {
								if(irCommSyncStateIndexTemp>1) {
									irCommSyncStateIndexTemp = 1;
									//updateRedLed(0);
								}
								irCommSyncStateTemp[irCommSyncStateIndexTemp] = 4;
								irCommSyncStateIndexTemp++;

								//irCommShiftCount = irCommShiftCount;
								irCommRxPeakHighToLow = 0;
								irCommRxStartBitDetected = 0;
								irCommSecondBitSkipped = 0;
								irCommShiftCounter = 0;
								irCommRxBitCount = 0;	
								irCommRxCrc = 0;	
								irCommRxByte = 0;
								irCommState = IRCOMM_RX_SYNC_SIGNAL;
							} else {
								currentProx = 0;
								adcSaveDataTo = SKIP_SAMPLE;
								adcSamplingState = 0;
								irCommMode=IRCOMM_MODE_SENSORS_SAMPLING;								
								irCommState = IRCOMM_RX_IDLE_STATE;

								irCommSyncStateIndexTemp++;
							}							
						} else {							
							currentProx = 0;
							adcSaveDataTo = SKIP_SAMPLE;
							adcSamplingState = 0;
							irCommMode=IRCOMM_MODE_SENSORS_SAMPLING;							
							irCommState = IRCOMM_RX_IDLE_STATE;	

							if(irCommSyncStateIndexTemp>1) {
								irCommSyncStateIndexTemp = 1;
								//updateRedLed(0);
							}
							irCommSyncStateTemp[irCommSyncStateIndexTemp] = 11;
							irCommSyncStateIndexTemp++;

							//irCommState = IRCOMM_RX_DEBUG;
							//irCommAdcRxState = 12;
							//updateRedLed(0);
							//break;

							resetDebugVariables();				
							break;
						}
					}
				} else {
					if(irCommRxStartBitDetected==1) {
						if(irCommSwitchCount==2) {
							if(irCommSyncStateIndexTemp>1) {
								irCommSyncStateIndexTemp = 1;
								//updateRedLed(0);
							}
							irCommSyncStateTemp[irCommSyncStateIndexTemp] = 5;
							irCommSyncStateIndexTemp++;

							irCommShiftCount = IRCOMM_SAMPLING_WINDOW/2 + irCommShiftCount;
							irCommRxPeakHighToLow = 0;
							irCommRxStartBitDetected = 0;
							irCommSecondBitSkipped = 0;
							irCommShiftCounter = 0;
							irCommRxBitCount = 0;	
							irCommRxCrc = 0;	
							irCommRxByte = 0;
							irCommState = IRCOMM_RX_SYNC_SIGNAL;
						} else if(irCommSwitchCount==1) {
							if(irCommSyncStateIndexTemp>1) {
								irCommSyncStateIndexTemp = 1;
								//updateRedLed(0);
							}
							irCommSyncStateTemp[irCommSyncStateIndexTemp] = 8;
							irCommSyncStateIndexTemp++;

							irCommShiftCount = 0;
							irCommRxPeakHighToLow = 0;
							irCommRxStartBitDetected = 0;
							irCommSecondBitSkipped = 0;
							irCommShiftCounter = 0;
							irCommRxBitCount = 0;	
							irCommRxCrc = 0;	
							irCommRxByte = 0;
							irCommState = IRCOMM_RX_WAITING_BIT;
						} else {
							irCommRxStartBitDetected = 0;
							currentProx = 0;
							adcSaveDataTo = SKIP_SAMPLE;
							adcSamplingState = 0;
							irCommMode=IRCOMM_MODE_SENSORS_SAMPLING;
							irCommState = IRCOMM_RX_IDLE_STATE;	

							if(irCommSyncStateIndexTemp>1) {
								irCommSyncStateIndexTemp = 1;
								//updateRedLed(0);
							}
							irCommSyncStateTemp[irCommSyncStateIndexTemp] = 13;
							irCommSyncStateIndexTemp++;

							//irCommState = IRCOMM_RX_DEBUG;
							//irCommAdcRxState = 12;
							//updateRedLed(0);
							//break;

							resetDebugVariables();				
							break;
						}
					} else {
						if(irCommSwitchCount==2) {
							if(irCommSyncStateIndexTemp>1) {
								irCommSyncStateIndexTemp = 1;
								//updateRedLed(0);
							}
							irCommSyncStateTemp[irCommSyncStateIndexTemp] = 1;
							irCommSyncStateIndexTemp++;

							irCommShiftCount = IRCOMM_SAMPLING_WINDOW/2 + irCommShiftCount;
							irCommRxPeakHighToLow = 0;
							irCommRxStartBitDetected = 0;
							irCommSecondBitSkipped = 0;
							irCommShiftCounter = 0;
							irCommRxBitCount = 0;	
							irCommRxCrc = 0;	
							irCommRxByte = 0;
							irCommState = IRCOMM_RX_SYNC_SIGNAL;
						} else if(irCommSwitchCount==1) {
							if(irCommRxStartPeakDuration > IRCOMM_SAMPLING_WINDOW/2) {
								currentProx = 0;
								adcSaveDataTo = SKIP_SAMPLE;
								adcSamplingState = 0;
								irCommMode=IRCOMM_MODE_SENSORS_SAMPLING;								
								irCommState = IRCOMM_RX_IDLE_STATE;

								if(irCommSyncStateIndexTemp>1) {
									irCommSyncStateIndexTemp = 1;
									//updateRedLed(0);
								}
								irCommSyncStateTemp[irCommSyncStateIndexTemp] = 14;
								irCommSyncStateIndexTemp++;

								//irCommState = IRCOMM_RX_DEBUG;
								//irCommAdcRxState = 12;
								//updateRedLed(0);
								//break;

								resetDebugVariables();				
								break;
							} else {
								if(irCommSyncStateIndexTemp>1) {
									irCommSyncStateIndexTemp = 1;
									//updateRedLed(0);
								}
								irCommSyncStateTemp[irCommSyncStateIndexTemp] = 7;
								irCommSyncStateIndexTemp++;

								irCommShiftCount = IRCOMM_SAMPLING_WINDOW;
								irCommRxPeakHighToLow = 0;
								irCommRxStartBitDetected = 0;
								irCommSecondBitSkipped = 0;
								irCommShiftCounter = 0;
								irCommRxBitCount = 0;	
								irCommRxCrc = 0;	
								irCommRxByte = 0;
								irCommState = IRCOMM_RX_SYNC_SIGNAL;
							}
						} else {
							currentProx = 0;
							adcSaveDataTo = SKIP_SAMPLE;
							adcSamplingState = 0;
							irCommMode=IRCOMM_MODE_SENSORS_SAMPLING;							
							irCommState = IRCOMM_RX_IDLE_STATE;

							if(irCommSyncStateIndexTemp>1) {
								irCommSyncStateIndexTemp = 1;
								//updateRedLed(0);
							}
							irCommSyncStateTemp[irCommSyncStateIndexTemp] = 15;
							irCommSyncStateIndexTemp++;
							
							//irCommState = IRCOMM_RX_DEBUG;
							//irCommAdcRxState = 12;
							//updateRedLed(0);
							//break;

							resetDebugVariables();				
							break;				
						}
					}
				}
	
				if(irCommStateIndexTemp>13) {
					irCommStateIndexTemp = 13;
					//updateRedLed(0);
				}
				irCommStateTemp[irCommStateIndexTemp] = irCommState;
				irCommStateIndexTemp++;
				
				if(irCommShiftCountFinalIndexTemp>1) {
					irCommShiftCountFinalIndexTemp = 1;
					//updateRedLed(0);
				}
				irCommShiftCountFinalTemp[irCommShiftCountFinalIndexTemp] = irCommShiftCount;
				irCommShiftCountFinalIndexTemp++;
				
				if(irCommRxStartBitDetectedIndexTemp>1) {
					irCommRxStartBitDetectedIndexTemp = 1;
					//updateRedLed(0);
				}
				irCommRxStartBitDetectedTemp[irCommRxStartBitDetectedIndexTemp] = irCommRxStartBitDetected;
				irCommRxStartBitDetectedIndexTemp++;			

				/*
				updateBlueLed(0);
				// transmit debug information
				if(DEBUG_START_BIT_STATE) {
					irCommSendValues = 0;						
					while(irCommSendValues==0);	// wait for the start from the uart (computer)
					usart0Transmit(0xFF, 1);
					usart0Transmit(irCommRxMaxSensor,1);
					usart0Transmit(irCommRxMaxDiff&0xFF,1);
					usart0Transmit(irCommRxMaxDiff>>8,1);
					usart0Transmit(irCommProxMean&0xFF,1);
					usart0Transmit(irCommProxMean>>8,1);
					usart0Transmit(irCommSwitchCount,1);
					for(i=0; i<IRCOMM_SAMPLING_WINDOW; i++) {
						irCommTempValue = irCommMaxSensorSignal[i];
						usart0Transmit(irCommTempValue&0xFF,1);
						usart0Transmit(irCommTempValue>>8,1);
						irCommTempValue = irCommMaxSensorSignalTemp[i];
						usart0Transmit(irCommTempValue&0xFF,1);
						usart0Transmit(irCommTempValue>>8,1);
					}
					usart0Transmit(irCommMaxSensorValueCurrTemp&0xFF,1);
					usart0Transmit(irCommMaxSensorValueCurrTemp>>8,1);
					usart0Transmit(irCommMinSensorValueCurrTemp&0xFF,1);
					usart0Transmit(irCommMinSensorValueCurrTemp>>8,1);
					usart0Transmit(irCommShiftCountTemp,1);
					usart0Transmit(irCommShiftCount,1);
					usart0Transmit(irCommStartDiffTemp&0xFF,1);
					usart0Transmit(irCommStartDiffTemp>>8,1);
					usart0Transmit(irCommComputeShift,1);
					usart0Transmit(irCommRxPeakHighToLow,1);
					usart0Transmit(irCommRxStartPeakDuration,1);
					usart0Transmit(irCommRxStartBitDetected,1);
					usart0Transmit(irCommState,1);					
				}
				updateBlueLed(255);
				*/

				break;
				
			case IRCOMM_RX_SYNC_SIGNAL:				
				break;

			case IRCOMM_RX_WAITING_BIT:
				break;

			case IRCOMM_RX_READ_BIT:
				// extract signal from the sensor with higher amplitude and compute the signal mean
				irCommProxSum = 0;
				irCommTempMin = 1024;
				irCommTempMax = 0;
				for(i=0; i<IRCOMM_SAMPLING_WINDOW; i++) {
					irCommMaxSensorSignal[i] = irCommProxValuesCurr[irCommRxMaxSensor+i*8];
					irCommProxSum += irCommMaxSensorSignal[i];
					if(irCommTempMin > irCommMaxSensorSignal[i]) {
						irCommTempMin = irCommMaxSensorSignal[i];
					}
					if(irCommTempMax < irCommMaxSensorSignal[i]) {
						irCommTempMax = irCommMaxSensorSignal[i];
					}
				}

				if((irCommTempMax-irCommTempMin) < IRCOMM_DETECTION_AMPLITUDE_THR) {	// error...no significant signal perceived					
					currentProx = 0;
					adcSaveDataTo = SKIP_SAMPLE;
					adcSamplingState = 0;
					irCommMode=IRCOMM_MODE_SENSORS_SAMPLING;
					irCommState = IRCOMM_RX_IDLE_STATE;

					//irCommState = IRCOMM_RX_DEBUG;
					//irCommAdcRxState = 12;
					//updateGreenLed(0);
					//break;

					resetDebugVariables();
					break;
				}

				irCommProxMean = (int)(irCommProxSum / IRCOMM_SAMPLING_WINDOW);

				// substract mean from signal
				for(i=0; i<IRCOMM_SAMPLING_WINDOW; i++) {
					irCommMaxSensorSignal[i] -= irCommProxMean;

					if(irCommBitsSignalIndexTemp>199) {
						irCommBitsSignalIndexTemp = 199;
						//updateRedLed(0);
					}
					irCommBitsSignalTemp[irCommBitsSignalIndexTemp] = irCommMaxSensorSignal[i];
					irCommBitsSignalIndexTemp++;
				}
						
				// start counting number of switch around mean signal
				if(irCommMaxSensorSignal[0] > 0) {
					irCommSignalState = 1;
				} else {
					irCommSignalState = -1;
				}
				irCommSwitchCount = 0;
				for(i=1; i<IRCOMM_SAMPLING_WINDOW; i++) {
					if(irCommMaxSensorSignal[i] > 0) {
						if(irCommSignalState < 0) {
							irCommSignalState = 1;
							irCommSwitchCount++;
						}
					} else {
						if(irCommSignalState > 0) {
							irCommSignalState = -1;
							irCommSwitchCount++;
						}
					}
				}
				// check whether we received either a "0" or a "1"
				if(irCommSwitchCount >= (IRCOMM_BIT0_SWITCH_COUNT-2)) {
					irCommRxBitReceived[irCommRxBitCount] = 0;
					if(irCommRxBitCount<8) {	// do not consider the crc for byte interpretation
						irCommRxByte = irCommRxByte<<1;	// bit0, only shift
					}
				} else if(irCommSwitchCount >= (IRCOMM_BIT1_SWITCH_COUNT-1)) {
					irCommRxBitReceived[irCommRxBitCount] = 1;
					if(irCommRxBitCount<8) {	// do not consider the crc for byte interpretation
						irCommRxCrc++;
						irCommRxByte = irCommRxByte<<1;	// bit1, shift and add 1
						irCommRxByte += 1;
					}
				} else {	// error...no significant signal perceived
					//irCommRxBitReceived[irCommRxBitCount] = 0xFF;
					//updateRedLed(0);
					currentProx = 0;
					adcSaveDataTo = SKIP_SAMPLE;
					adcSamplingState = 0;
					irCommMode=IRCOMM_MODE_SENSORS_SAMPLING;					
					irCommState = IRCOMM_RX_IDLE_STATE;

					//irCommState = IRCOMM_RX_DEBUG;
					//irCommAdcRxState = 12;
					//break;

					resetDebugVariables();
					break;

				}

				if(irCommRxBitReceivedIndexTemp>9) {
					irCommRxBitReceivedIndexTemp = 9;
					//updateRedLed(0);
				}
				irCommRxBitReceivedTemp[irCommRxBitReceivedIndexTemp] = irCommRxBitReceived[irCommRxBitCount];
				irCommRxBitReceivedIndexTemp++;

				//setGreenLed(irCommRxBitCount, 1);

				irCommRxBitCount++;
				if(irCommRxBitCount == 10) {	// received 8 bit of data + 2 bit of crc
					irCommState = IRCOMM_RX_CHECK_CRC;

					if(irCommStateIndexTemp>13) {
						irCommStateIndexTemp = 13;
						//updateRedLed(0);
					}
					irCommStateTemp[irCommStateIndexTemp] = irCommState;
					irCommStateIndexTemp++;

				} else {
					irCommState = IRCOMM_RX_WAITING_BIT;

					if(irCommStateIndexTemp>13) {
						irCommStateIndexTemp = 13;
						//updateRedLed(0);
					}
					irCommStateTemp[irCommStateIndexTemp] = irCommState;
					irCommStateIndexTemp++;

				}							
				break;

			case IRCOMM_RX_CHECK_CRC:
				irCommRxCrcError = (irCommRxCrc + (irCommRxBitReceived[8]<<1) + irCommRxBitReceived[9])&0x03;
				if(irCommRxCrcError==0) {
					irCommRxLastDataReceived = irCommRxByte;
					irCommRxReceivingSensor = irCommRxMaxSensor;
					irCommRxDataAvailable = 1;
					//updateBlueLed(0);
					usart0Transmit(irCommRxByte,1);		
					//updateBlueLed(255);			
				}
												
				currentProx = 0;
				adcSaveDataTo = SKIP_SAMPLE;
				adcSamplingState = 0;
				irCommMode=IRCOMM_MODE_SENSORS_SAMPLING;				
								
				/*
				if((irCommRxByte!=irCommRxByteExpected) || (irCommRxCrcError!=0)) {	// if there is an error
					// debug...
					irCommState = IRCOMM_RX_DEBUG;
					irCommAdcRxState = 12;
					updateRedLed(0);
					updateGreenLed(0);
					updateBlueLed(0);
					break;

				} else {
					irCommState = IRCOMM_RX_IDLE_STATE;
				}

				if(irCommRxByteExpected == 255) {
					irCommRxByteExpected = 0;
					irCommRxSequenceCount++;
				} else {
					irCommRxByteExpected++;
				}				
				*/

				/*
				if(abs(irCommRxPrevDataReceived-irCommRxLastDataReceived)>10) {
					irCommState = IRCOMM_RX_DEBUG;
					irCommAdcRxState = 12;
					updateRedLed(200);
					updateGreenLed(200);
					updateBlueLed(200);
					break;
				} else {
					irCommRxPrevDataReceived = irCommRxLastDataReceived;
				}
				*/

				resetDebugVariables();

				irCommState = IRCOMM_RX_IDLE_STATE;

				break;

			case IRCOMM_RX_DEBUG:								
				irCommSendValues = 0;						
				while(irCommSendValues==0);	// wait for the start from the uart (computer)
				usart0Transmit(0xFF, 1);
				usart0Transmit(irCommRxMaxSensorTemp[0],1);
				usart0Transmit(irCommRxMaxSensorTemp[1],1);
				usart0Transmit(irCommRxMaxDiffTemp[0]&0xFF,1);
				usart0Transmit(irCommRxMaxDiffTemp[0]>>8,1);
				usart0Transmit(irCommRxMaxDiffTemp[1]&0xFF,1);
				usart0Transmit(irCommRxMaxDiffTemp[1]>>8,1);
				usart0Transmit(irCommProxMeanTemp[0]&0xFF,1);
				usart0Transmit(irCommProxMeanTemp[0]>>8,1);
				usart0Transmit(irCommProxMeanTemp[1]&0xFF,1);
				usart0Transmit(irCommProxMeanTemp[1]>>8,1);
				usart0Transmit(irCommSwitchCountTemp[0],1);
				usart0Transmit(irCommSwitchCountTemp[1],1);
				for(i=0; i<IRCOMM_SAMPLING_WINDOW*2; i++) {
					irCommTempValue = irCommMaxSensorSignalTemp[i];
					usart0Transmit(irCommTempValue&0xFF,1);
					usart0Transmit(irCommTempValue>>8,1);
					irCommTempValue = irCommMaxSensorSignalFiltTemp[i];
					usart0Transmit(irCommTempValue&0xFF,1);
					usart0Transmit(irCommTempValue>>8,1);
				}
				usart0Transmit(irCommMaxSensorValueCurrTemp[0]&0xFF,1);
				usart0Transmit(irCommMaxSensorValueCurrTemp[0]>>8,1);
				usart0Transmit(irCommMaxSensorValueCurrTemp[1]&0xFF,1);
				usart0Transmit(irCommMaxSensorValueCurrTemp[1]>>8,1);
				usart0Transmit(irCommMinSensorValueCurrTemp[0]&0xFF,1);
				usart0Transmit(irCommMinSensorValueCurrTemp[0]>>8,1);
				usart0Transmit(irCommMinSensorValueCurrTemp[1]&0xFF,1);
				usart0Transmit(irCommMinSensorValueCurrTemp[1]>>8,1);
				usart0Transmit(irCommShiftCountTemp[0],1);
				usart0Transmit(irCommShiftCountTemp[1],1);
				usart0Transmit(irCommShiftCountFinalTemp[0],1);
				usart0Transmit(irCommShiftCountFinalTemp[1],1);
				usart0Transmit(irCommStartDiffTemp[0]&0xFF,1);
				usart0Transmit(irCommStartDiffTemp[0]>>8,1);
				usart0Transmit(irCommStartDiffTemp[1]&0xFF,1);
				usart0Transmit(irCommStartDiffTemp[1]>>8,1);
				usart0Transmit(irCommComputeShiftTemp[0],1);
				usart0Transmit(irCommComputeShiftTemp[1],1);
				usart0Transmit(irCommRxPeakHighToLowTemp[0],1);
				usart0Transmit(irCommRxPeakHighToLowTemp[1],1);
				usart0Transmit(irCommRxStartPeakDurationTemp[0],1);
				usart0Transmit(irCommRxStartPeakDurationTemp[1],1);
				usart0Transmit(irCommRxStartBitDetectedTemp[0],1);
				usart0Transmit(irCommRxStartBitDetectedTemp[1],1);
				usart0Transmit(irCommSyncStateTemp[0],1);
				usart0Transmit(irCommSyncStateTemp[1],1);
				for(i=0; i<14; i++) {
					usart0Transmit(irCommStateTemp[i],1);
				}
				for(i=0; i<200; i++) {
					irCommTempValue = irCommBitsSignalTemp[i];
					usart0Transmit(irCommTempValue&0xFF,1);
					usart0Transmit(irCommTempValue>>8,1);
				}
				for(i=0; i<10; i++) {
					usart0Transmit(irCommRxBitReceivedTemp[i],1);
				}
				usart0Transmit(irCommRxCrc,1);
				usart0Transmit(irCommRxCrcError,1);
				usart0Transmit(irCommRxByte,1);
				usart0Transmit(irCommRxByteExpected,1);				

				irCommState = IRCOMM_RX_STOP;
				break;
	
			case IRCOMM_RX_STOP:
				break;
					
		}

	} else if(irCommMode==IRCOMM_MODE_TRANSMIT) {

		switch(irCommState) {
			case IRCOMM_TX_IDLE_STATE:					
				break;

			case IRCOMM_TX_PREPARE_TRANSMISSION:				
				//updateBlueLed(255);
				//updateBlueLed(0);
				irCommTickCounter = getTime100MicroSec()-irCommTickCounter2;
				irCommTickCounter2 = getTime100MicroSec();
				irCommTxBitToTransmit[0] = 2;	// start bit 1
				irCommTxBitToTransmit[1] = 3;	// start bit 2 
				irCommTxCrc = 0;
				for(i=0; i<8; i++) {
					irCommTempValue = (irCommTxByte>>i)&0x01;
					irCommTxBitToTransmit[9-i] = irCommTempValue;
					if(irCommTempValue==1) {
						irCommTxCrc++;
					}
					//irCommTxBitToTransmit[9-i] = 2;
				}
				irCommTxCrc = irCommTxCrc % 4;
				irCommTxCrc = 4 - irCommTxCrc;
				irCommTxBitToTransmit[10] = (irCommTxCrc>>1)&0x01;
				irCommTxBitToTransmit[11] = irCommTxCrc&0x01;	
				irCommTxBitCount = 0;							
				irCommTxPulseState = 0;	
				irCommState = IRCOMM_TX_COMPUTE_TIMINGS;				
				break;

			case IRCOMM_TX_COMPUTE_TIMINGS:
				//updateBlueLed(255);
				if(irCommTxBitToTransmit[irCommTxBitCount] == 3) {
					//updateBlueLed(0);
					irCommTxDuration = IRCOMM_BIT_START2_DURATION;					
					irCommTxSwitchCount = IRCOMM_BIT_START2_SWITCH_COUNT;
				} else if(irCommTxBitToTransmit[irCommTxBitCount] == 2) {
					//updateBlueLed(0);
					irCommTxDuration = IRCOMM_BIT_START1_DURATION;					
					irCommTxSwitchCount = IRCOMM_BIT_START1_SWITCH_COUNT;
				} else if(irCommTxBitToTransmit[irCommTxBitCount] == 1) {
					irCommTxDuration = IRCOMM_BIT1_DURATOIN;					
					irCommTxSwitchCount = IRCOMM_BIT1_SWITCH_COUNT;
				} else {
					irCommTxDuration = IRCOMM_BIT0_DURATION;						
					irCommTxSwitchCount = IRCOMM_BIT0_SWITCH_COUNT;
				}
				if(irCommTxBitCount == 0) {
					//PORTA = irCommTxSensorMask;
					if(irCommTxSensorGroup==0) {
						PORTA = 0xAA;
					} else {
						PORTA = 0x55;
					}					
					irCommTxPulseState = 1;
				}
				irCommTxDurationCycle = 0;
				irCommTxSwitchCounter = 0;						
				irCommState = IRCOMM_TX_TRANSMIT_BIT;
				irCommAdcTxState = IRCOMM_TX_ADC_TRANSMISSION_SEQ1;
				break;

			case IRCOMM_TX_TRANSMIT_BIT:	// wait for bit to be transmitted
				break;

		}

	}

}

//void irCommSendData(unsigned char value, unsigned char sensorMask) {
//	irCommTxByte = value;
//	irCommTxByteEnqueued = 1;
//	irCommTxSensorMask = sensorMask;
//	irCommEnabledNext = IRCOMM_MODE_TRANSMIT;
//}

void irCommSendData(unsigned char value) {
	irCommTxByte = value;
	irCommTxByteEnqueued = 1;
	irCommEnabledNext = IRCOMM_MODE_TRANSMIT;
}

unsigned char irCommDataSent() {
	if(irCommTxByteEnqueued==1) {
		return 0;
	} else {
		return 1;
	}
}

unsigned char irCommDataAvailable() {
	return irCommRxDataAvailable;
}

unsigned char irCommReadData() {
	irCommRxDataAvailable = 0;
	return irCommRxLastDataReceived;
}

signed char irCommReceivingSensor() {
	return irCommRxReceivingSensor;
}

signed int getBearing(unsigned char sensor) {
	switch(sensor) {
		case 0:
			return 0;
		case 1:
			return -45;
		case 2:
			return -90;
		case 3:
			return -135;
		case 4:
			return 180;
		case 5:
			return 135;
		case 6:
			return 90;
		case 7:
			return 45;
		default:
			return -1;
	}
}



