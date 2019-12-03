#include <adc.h>
#include <behaviors.h>
#include <constants.h>
#include <ir_remote_control.h>
#include <irCommunication.h>
#include <leds.h>
#include <mirf.h>
#include <motors.h>
#include <nRF24L01.h>
#include <ports_io.h>
#include <sensors.h>
#include <speed_control.h>
#include <spi.h>
#include <twimaster.h>
#include <usart.h>
#include <utility.h>
#include <variables.h>
#include <math.h>
unsigned long int startTime = 0, endTime = 0;
unsigned char prevSelector = 0;

void setup() {

  initPeripherals();
  calibrateSensors();
  initBehaviors();
  disableObstacleAvoidance();
  disableCliffAvoidance();
  startTime = getTime100MicroSec();
  pwm_red = rand() % 255;
      pwm_green = rand() % 255;
      pwm_blue = rand() % 255;
      updateRedLed(pwm_red);
            updateGreenLed(pwm_green);
            updateBlueLed(pwm_blue);
}
//proximityResultLinear[8]->Leitura linear dos sensores: 0 - 7 sensores laterais onde 0:N,1:NE,2:E,3:SE,4:S,5:SW;6:W;7:NW
//proximityResult[12] os sensores 8,9,10,11 são os inferiores(tem q descobrir qual eh qual)
void loop() {

  currentSelector = getSelector();	          // update selector position

  readAccelXYZ();			          // update accelerometer values to compute the angle

  computeAngle();
	
  endTime = getTime100MicroSec();
  if((endTime-startTime) >= (PAUSE_1_SEC/2)) {
    usart0Transmit('\n',0);
    usart0Transmit('a',0);
    usart0Transmit('\n',0);
    for(int i = 0;i<8;i++){
      printInt(proximityResultLinear[i]);
      usart0Transmit('\n',0);
    }
    usart0Transmit('b',0);
    usart0Transmit('\n',0);
    for(int i = 0;i<4;i++){
      printInt(getDist(90*i));
      usart0Transmit('\n',0);
    }
    usart0Transmit('c',0);
    usart0Transmit('\n',0);
    for(int i = 0;i<12;i++){
      printInt(proximityResult[i]);
      usart0Transmit('\n',0);
    }
    startTime = getTime100MicroSec();
  }

  handleIRRemoteCommands();

  handleRFCommands();
}
void printInt(int num){
  if(num == 0){
    usart0Transmit('0',0);
    return;
  }
  if(num < 0){
    usart0Transmit('-',0);
    num = -num;
  }
  int its = (int)(log10(num));
  while(its != -1){
    usart0Transmit((char)((int)(num/pow(10,its)) + '0'),0);
    num = num - (int)(num/pow(10,its))*pow(10,its--);
  }
}
int getDist(int localAngle) { // 0 esq 90 up 180 dir 270 down
  if (localAngle % 45 != 0)
    return 0;
  int val = ((localAngle / 45) - 2) % 8;
  if (val < 0)
    val = (8 + val);
  int valB = ((val - 1 < 0) ? (8 + val - 1) : (val - 1)) % 8;
  int valA = (val + 1) % 8;

  return ((proximityResultLinear[valB] + proximityResultLinear[valB]) * 0.5 + proximityResultLinear[val]) / 2;

}
