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


#define THRBW  380
#define THRBWL  270
#define THRBWH 490
#define PASSBW 25
#define DOTDIST  50
#define COMDIST 70
#define WAITDIST 50
#define NEARCOLDIST 200
#define SPEED  5
#define SPEED_RAT 1.5
#define GRIDWIDTH 20
#define N 6
#define M 21


unsigned long int startTime = 0, endTime = 0;
int begiS = 0;
int globalAngle = 0; // 0 esq 90 up 180 dir 270 down
bool rotationH = false; //  se a rotação esta acontecendo
int rotationS = 0; // estado da rotacao
bool translationH = false;
bool translationS = true;
int x, y;
bool lastIsBlack[4];
char curve = 0;
int curveX[N][M] =
{ {8,   9,   10, 10, 10, 9,   8,   8,   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  {2,   3,   4,   4,    4,  3,   2,   2,   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  {10, 10, 10, 10, 9, 8, 7, 8, 9, 10, 10, 10, -1, -1, -1, -1, -1, -1, -1,-1,-1},
  {6, 6, 6, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  {2, 2, 2, 2, 3, 4, 5, 6, 5, 4, 3, 2, 2, 2, -1, -1, -1, -1, -1, -1, -1},
  {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, -1}
}
;
int curveY[N][M] =
{ {10, 10, 10, 11, 12, 12, 12, 11,   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  {10, 10, 10, 11, 12, 12, 12, 11,   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  {6,   5,   4,   3,   3, 3, 3, 3, 3, 3,   4,   5, -1, -1, -1, -1, -1, -1, -1,-1,-1},
  {7, 6 , 5 , 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  {6, 5, 4 , 3 , 3, 3, 3 , 3 , 3 , 3, 3, 3, 4, 5, -1, -1, -1, -1, -1, -1, -1},
  {5 , 5, 5, 5, 5 , 5 , 5 , 5, 5, 5,   5,   5,   5, 5 , 5, 5, 5, 5, 5, 5, -1}
}
;
int curveItInit[N];
int curvesLen[N];
int curveIt;
int waitBots;
int angOff = 0;
int count = 0;
int sum = 0;
int colPlaces[M];
int colFrom[M];
//funcoes basicas do arduino
void setup() {
  // put your setup code here, to run once:
  initPeripherals();
  //initAccelerometer();
  initBehaviors();
  calibrateSensors();
  startTime = getTime100MicroSec();
  //initMotors();
  disableObstacleAvoidance();
  disableCliffAvoidance();
  curveIt = curveItInit[0];
  for (int i = 0; i < 4; i++) {
    lastIsBlack[4] = false;
  }
  for(int i = 0;i<N;i++){
    int j = 0;
    while(curveX[i][j] >= 0){
      curvesLen[i]++;
      j++;
    }
  }
}


void loop() {
  handleMotorsWithSpeedController();
  handleIRRemoteCommands();
  handleRFCommands();
  if (begiS != 5) {
    startFun();
    //debugg();
  }
  else {
    cicle();
  }
  if (count < 100) {
    count++;
  }
  else {
    printInt(x);
    printInt(begiS);
    count = 0;
  }
}

//funções dos dois estados do robo: esta inicial e estado de ciclo
void startFun() {
  bool endStart = true;
  switch (begiS) {
    case -2:
      if (getDist(90) > 100)
        begiS = -1;
      break;
    case -1:
      if (getDist(90) < 50)
        begiS = 0;
      break;
    case 0://ir para frente ate quase sair do grid
      if ((getDist(0) > DOTDIST) && !isBlack(0) && !isBlack(3)) {
        setLeftSpeed(0);
        setRightSpeed(0);
        angOff = getAngle();
        begiS = 1;
      }
      else if (getDist(90) > COMDIST) {
        setLeftSpeed(0);
        setRightSpeed(0);
        if (irCommDataAvailable() == 1 && irCommReceivingSensor() == 0) {
          curve = irCommReadData();
          irCommSendData(42);
        }
      }
      else
        follow();
      break;
    case 1://rotating
      if (!rotate(90)) {
        begiS = 2;
        x = 0;
        y = 0;
        if (getSelector() == 5) {
          curve = N - 1;
          begiS = 3;
        }
        curveIt = curveItInit[curve];
      }
      break;
    case 2://esperando chegar outro
      if (getDist(180) > COMDIST) {
        if (irCommDataAvailable() == 1 && irCommReceivingSensor() == 2 && 42 == irCommReadData()) {
          begiS = 3;
        }
        else {
          irCommSendData(curve + 1);
        }

      }
      break;
    case 3:

      if (!mov(curveY[curve][curveIt], false))
        begiS = 4;
      break;
    case 4:
      if (!mov(curveX[curve][curveIt], true)) {
        begiS = 5;
        curveIt++; 
      }
      break;
  }
}

void genCol() {
  for (int i = 0; i < M; i++) {
    colPlaces[i] = 0;
    colFrom[i] = 0;
    if (curveX[curve][i] >= 0) {
      for (int j = 0; j < N; j++) {
        if (j != curve) {
          for (int k = 0; k < M; k++) {
            if ((curveX[j][k] == curveX[curve][i]) && (curveY[j][k] == curveY[curve][i])) {
              colFrom[i] = j;
              int lastCIt = i - 1; //pegando o indice anterior
              if (lastCIt < 0) {
                lastCIt = M;
                while (curveX[curve][lastCIt] < 0)
                  lastCIt--;
              }
              int lastJIt = k - 1;
              if (lastJIt < 0) {
                lastJIt = M;
                while (curveX[j][lastJIt] < 0)
                  lastJIt--;
              }
              if (curveX[j][lastJIt] == curveX[curve][lastCIt] && curveY[j][lastJIt] == curveY[curve][lastCIt]) {
                if (colPlaces[i] < 1){
                  colPlaces[i] = 1;//colisao no msm sentido
                }
              }
              else if (curveX[j][lastJIt] != curveX[curve][lastCIt] && curveY[j][lastJIt] != curveY[curve][lastCIt]) {
                if (colPlaces[i] < 2){
                  colPlaces[i] = 2;//colisao lateral
                }
              }
              else{
                colPlaces[i] = 3;//colisao frontal
              }


            }
          }
        }
      }
    }
  }
}
void cicle() {
  if (curveX[curve][curveIt] != x) {
    if (!mov(curveX[curve][curveIt], true)) {
      curveIt++;
    }
  }
  else if (curveY[curve][curveIt] != y) {
    if (!mov(curveY[curve][curveIt], false)) {
      curveIt++;
    }
  }
  else {
    curveIt++;
    translationH = false;
  }
  if (curveX[curve][curveIt] < 0)
    curveIt = 0;
  checkCol();
}
void checkCol(){
  switch(colPlaces[curveIt]){
    case 1:
      if(getDist(90) > NEARCOLDIST){
        setLeftSpeed(0);
        setRightSpeed(0);
      }
      break;
    case 2:
      if(getDist(90) > NEARCOLDIST){
        setLeftSpeed(0);
        setRightSpeed(0);
      }
      if((getDist(45) > NEARCOLDIST/2 || getDist(135) > NEARCOLDIST/2)&&
      (curvesLen[curve]<curvesLen[colFrom[curveIt]]||(curvesLen[curve]==curvesLen[colFrom[curveIt]]&&curve < colFrom[curveIt]))){
        setLeftSpeed(0);
        setRightSpeed(0);
      }
    break;
    case 3:
      if(getDist(90) > NEARCOLDIST &&!(isBlack(0)&&isBlack(3))){
        backward();
      }
    break;
  }
}

//funções para fazer as ações do robo
void printInt(int num) {
  if (num == 0) {
    usart0Transmit('0', 0);
    usart0Transmit('\n', 0);
    return;
  }
  if (num < 0) {
    usart0Transmit('-', 0);
    num = -num;
  }
  int its = (int)(log10(num));
  while (its != -1) {
    usart0Transmit((char)((int)(num / pow(10, its)) + '0'), 0);
    num = num - (int)(num / pow(10, its)) * pow(10, its--);
  }
  usart0Transmit('\n', 0);
}

void debugg() {
  endTime = getTime100MicroSec();
  if ((endTime - startTime) >= (PAUSE_1_SEC)) {
    usart0Transmit('\n', 0);
    //    for (int i = 0; i < 8; i++) {
    //      printInt(proximityResultLinear[i]);
    //      usart0Transmit('\n', 0);
    //    }
    //    usart0Transmit('b', 0);
    //    usart0Transmit('\n', 0);
    //    for (int i = 0; i < 12; i++) {
    //      printInt(proximityResult[i]);
    //      usart0Transmit('\n', 0);
    //    }
    usart0Transmit('a', 0);
    for (int i = 0; i < 4; i++) {
      printInt(isBlack(i));
    }
    startTime = getTime100MicroSec();
  }
}

bool isBlack(int sensorN) {
  if (!lastIsBlack[sensorN]) {
    if (proximityResult[sensorN + 8] <= THRBW - PASSBW) {
      lastIsBlack[sensorN] = true;
      return true;
    }
    else {
      lastIsBlack[sensorN] = false;
      return false;
    }
  }
  else {
    if (proximityResult[sensorN + 8] >= THRBW + PASSBW) {
      lastIsBlack[sensorN] = false;
      return false;
    }
    else {
      lastIsBlack[sensorN] = true;
      return true;
    }
  }
}

int getDist(int localAngle) { // 0 esq 90 up 180 dir 270 down
  if (localAngle % 45 != 0)
    return 0;
  int val = ((localAngle / 45) - 2) % 8;
  if (val < 0)
    val = (8 + val);
  //  int valB = ((val - 1 < 0) ? (8 + val - 1) : (val - 1)) % 8;
  //  int valA = (val + 1) % 8;

  return (proximityResult[val]);

}

void follow() { //considerando sensor 1(9) na esquerda e 2(10) na direita
  setLeftSpeed(SPEED);
  setRightSpeed(SPEED);
  if (!isBlack(1)) {
    setRightSpeed(0);
    //printInt(-123);
  }
  if (!isBlack(2)) {
    setLeftSpeed(0);
    //printInt(123);
  }
}
void backward(){//ré
  setLeftSpeed(-SPEED);
  setRightSpeed(-SPEED);
  if (!isBlack(2)) {
    setRightSpeed(0);
    //printInt(-123);
  }
  if (!isBlack(1)) {
    setLeftSpeed(0);
    //printInt(123);
  }
}
bool rotate(int angle) {
  if (!rotationH) {
    rotationH = true;
    switch (angle) {
      case 90:
        rotationS = 1;
        break;
      case -90:
        rotationS = 1;
        break;
      case 180:
        rotationS = 3;
        break;
      default:
        rotationH = false;
        rotationS = 0;
        return false;
        break;
    }
  }
  switch (rotationS) {
    case 1:
      if (angle > 0) {
        setLeftSpeed(SPEED / 2);
        setRightSpeed(-SPEED / 2);
      }
      else {
        setLeftSpeed(-SPEED / 2);
        setRightSpeed(SPEED / 2);
      }
      if ((angle > 0) ? (isBlack(0)) : (isBlack(3)))
        rotationS = 2;
      break;
    case 2:
      if (angle > 0) {
        setLeftSpeed(SPEED / 2);
        setRightSpeed(-SPEED / 2);
      }
      else {
        setLeftSpeed(-SPEED / 2);
        setRightSpeed(SPEED / 2);
      }
      if ((isBlack(1) || isBlack(2)) && (angle > 0) ? (!isBlack(0)) : (!isBlack(3))) {
        rotationS = 0;
        rotationH = false;
        setLeftSpeed(0);
        setRightSpeed(0);
        globalAngle = (globalAngle + angle + 360) % 360;
        return false;
      }
      break;
    case 3:
      setLeftSpeed(-SPEED / 2);
      setRightSpeed(-SPEED / 2);
      if (isBlack(0) && isBlack(3)) {
        rotationS = 4;
      }
      break;
    case 4:
      setLeftSpeed(-SPEED / 2);
      setRightSpeed(-SPEED / 2);
      if (!isBlack(0))
        rotationS = 5;
      if (!isBlack(3))
        rotationS = 8;
      break;
    case 5:
      setLeftSpeed(-SPEED / 2);
      setRightSpeed(SPEED / 2);
      if (!isBlack(1))
        rotationS = 6;
      break;
    case 6:
      setLeftSpeed(-SPEED / 2);
      setRightSpeed(SPEED / 2);
      if (!isBlack(2))
        rotationS = 7;
      break;
    case 7:
      setLeftSpeed(-SPEED / 2);
      setRightSpeed(SPEED / 2);
      if (isBlack(2)) {
        rotationS = 0;
        rotationH = false;
        setLeftSpeed(0);
        setRightSpeed(0);
        globalAngle = (globalAngle + 180) % 360;
        return false;
      }
      break;
    case 8:
      setLeftSpeed(SPEED / 2);
      setRightSpeed(-SPEED / 2);
      if (!isBlack(2))
        rotationS = 9;
      break;
    case 9:
      setLeftSpeed(SPEED / 2);
      setRightSpeed(-SPEED / 2);
      if (!isBlack(1))
        rotationS = 10;
      break;
    case 10:
      setLeftSpeed(SPEED / 2);
      setRightSpeed(-SPEED / 2);
      if (isBlack(1)) {
        rotationS = 0;
        rotationH = false;
        setLeftSpeed(0);
        setRightSpeed(0);
        globalAngle = (globalAngle + 180) % 360;
        return false;
      }
      break;
  }
  return true;
}

bool mov(int coor, bool verthor) { //verthor false indica vertical, true indica horizontal
  if (!translationH) {
    translationH = true;
    translationS = true;
  }
  if (!verthor) {
    if (y < coor && globalAngle != 90) {
      switch (globalAngle) {
        case 0:
          rotate(90);
          break;
        case 180:
          rotate(-90);
          break;
        case 270:
          rotate(180);
          break;
      }
      return true;
    }
    if (y > coor && globalAngle != 270) {
      switch (globalAngle) {
        case 0:
          rotate(-90);
          break;
        case 180:
          rotate(90);
          break;
        case 90:
          rotate(180);
          break;
      }
      return true;
    }
    follow();
    if (translationS) {
      if (isBlack(0) && isBlack(3)) {
        translationS = false;
      }
    }
    else {
      if (!isBlack(0) && !isBlack(3)) {
        translationS = true;
        if (globalAngle == 90)
          y++;
        else
          y--;
      }
    }
    if (coor == y) {
      translationH = false;
      setLeftSpeed(0);
      setRightSpeed(0);
      return false;
    }
  }
  else {
    if (x < coor && globalAngle != 180) {
      switch (globalAngle) {
        case 0:
          rotate(180);
          break;
        case 90:
          rotate(90);
          break;
        case 270:
          rotate(-90);
          break;
      }
      return true;
    }
    if (x > coor && globalAngle != 0) {
      switch (globalAngle) {
        case 180:
          rotate(180);
          break;
        case 90:
          rotate(-90);
          break;
        case 270:
          rotate(90);
          break;
      }
      return true;
    }
    follow();
    if (translationS) {
      if (isBlack(0) && isBlack(3)) {
        translationS = false;
      }
    }
    else {
      if (!isBlack(0) && !isBlack(3)) {
        translationS = true;
        if (globalAngle == 180)
          x++;
        else
          x--;
      }
    }
    if (coor == x) {
      translationH = false;
      setLeftSpeed(0);
      setRightSpeed(0);
      return false;
    }
  }
  if (getDist(90) >= (DOTDIST)) {
    setLeftSpeed(0);
    setRightSpeed(0);
    irCommSendData(11);
  }

  return true;

}

int getAngle() { // left = 0, ri = 180
  return (900 - currentAngle - angOff) % 360;
}
