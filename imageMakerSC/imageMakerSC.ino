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


#define THRBW  390
#define THRBWL  270
#define THRBWH 520
#define PASSBW 7
#define DOTDIST  100
#define COMDIST 130
#define WAITDIST 50
#define NEARCOLDIST 200
#define COLDIST 70
#define SPEED  5
#define SPEED_RAT 1.5
#define GRIDWIDTH 20
#define N 4//6
#define M 21
#define COMMPORTSN 10

/*
  bob
  0:
  x: 8   9   10 10 10 9   8   8   -1
  y: 10 10 10 11 12 12 12 11 -1

  1:
  x: 2   3   4   4    4  3   2   2   -1
  y: 10 10 10 11 12 12 12 11  -1

  2:
  x: 10 10 10 10 9 8 7 6 7 8 9 10 10 10 -1
  y: 6   5   4   3   3 3 3 3 3 3 3 3   4   5   -1

  3:
  x: 6 6 6 6 -1
  y: 7 6 5 6 -1
  4:
  x: 2 2 2 2 3 4 5 6 5 4 3 2 2 2 -1
  y: 6 5 4 3 3 3 3 3 3 3 3 3 4 5 -1

  5:
  x: 1 2 3 4 5 6 7 8 9 10 11 10 9 8 7 6 5 4 3 2 -1
  y: 5 5 5 5 5 5 5 5 5 5   5   5   5 5 5 5 5 5 5 5 -1
*/
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
char lastCurve = 0;
//caminho com colisão frontal complexa
//int curveX[N][M] =
//{
//  //{8,   9,   10, 10, 10, 9,   8,   8,   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
//  //{2,   3,   4,   4,    4,  3,   2,   2,   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
//  {10, 10, 10, 10, 9, 8, 7,6 ,5 , 6,7, 8, 9, 10, 10, 10, -1, -1, -1, -1,  -1},
//  {6, 6, 6, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
//  {2, 2, 2, 2, 3, 4, 5, 6,7,6, 5, 4, 3, 2, 2, 2, -1, -1, -1, -1, -1},
//  {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, -1}
//}
//;
//int curveY[N][M] =
//{
//  //{10, 10, 10, 11, 12, 12, 12, 11,   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
//  //{10, 10, 10, 11, 12, 12, 12, 11,   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
//  {6,   5,   4,   3,   3, 3,3,3,3,3, 3, 3, 3, 3,   4,   5, -1, -1, -1, -1, -1},
//  {7, 6 , 5 , 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
//  {6, 5, 4 , 3 , 3, 3, 3,3,3 , 3 , 3 , 3, 3, 3, 4, 5, -1, -1, -1, -1, -1},
//  {5 , 5, 5, 5, 5 , 5 , 5 , 5, 5, 5,   5,   5,   5, 5 , 5, 5, 5, 5, 5, 5, -1}
//}
//;

//caminho com colisão frontal simples
int curveX[N][M] =
{
  //{8,   9,   10, 10, 10, 9,   8,   8,   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  //{2,   3,   4,   4,    4,  3,   2,   2,   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  {10, 10, 10, 10, 9, 8, 7, 6, 7, 8, 9, 10, 10, 10, -1, -1, -1, -1, -1, -1, -1},
  {6, 6, 6, 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  {2, 2, 2, 2, 3, 4, 5, 6, 5, 4, 3, 2, 2, 2, -1, -1, -1, -1, -1, -1, -1},
  {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 10, 9, 8, 7, 6, 5, 4, 3, 2, -1}
}
;
int curveY[N][M] =
{
  //{10, 10, 10, 11, 12, 12, 12, 11,   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  //{10, 10, 10, 11, 12, 12, 12, 11,   -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  {6,   5,   4,   3,   3, 3, 3, 3, 3, 3, 3, 3,   4,   5, -1, -1, -1, -1, -1, -1, -1},
  {7, 6 , 5 , 6, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1},
  {6, 5, 4 , 3 , 3, 3, 3 , 3 , 3 , 3, 3, 3, 4, 5, -1, -1, -1, -1, -1, -1, -1},
  {5 , 5, 5, 5, 5 , 5 , 5 , 5, 5, 5,   5,   5,   5, 5 , 5, 5, 5, 5, 5, 5, -1}
}
;
int curveItInit[N];
int curvesLen[N];
int curveIt;
int lastCurveIt;
int waitBots;
int angOff = 0;
bool comm = false;
int count = 0;
int sum = 0;
int shakeCount = 0;
int colPlaces[N][M];
int colFrom[N][M];

char frontColStatus = 0;
bool missedInfo = false;

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
  for (int i = 0; i < N; i++) {
    int j = 0;
    while (curveX[i][j] >= 0) {
      curvesLen[i]++;
      j++;
    }
  }
  upColor();
  genCol();
}
void loop() {
  irCommTasks();
  handleMotorsWithSpeedController();
  //handleIRRemoteCommands();
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
    printInt(proximityResult[1 + 8]);
    printInt(proximityResult[2 + 8]);
    printInt(-1);
    count = 0;
  }
}
int lastRed = -1;
void upColor() {
  updateRedLed(200 - (((float)(curve)) / ((float)N)) * 200);
  updateGreenLed((((float)(curve)) / ((float)N)) * 200);
  updateBlueLed((((float)(curve)) / ((float)N)) * 200);
}
void genCol() {
  for (int cur = 0; cur < N; cur++) {
    for (int i = 0; i < M; i++) {
      colPlaces[cur][i] = 0;
      colFrom[cur][i] = -1;
      if (curveX[cur][i] >= 0) {
        for (int j = 0; j < N; j++) {
          if (j != cur) {
            for (int k = 0; k < M; k++) {
              if ((curveX[j][k] == curveX[cur][i]) && (curveY[j][k] == curveY[cur][i])) {
                colFrom[cur][i] = j;
                int lastCIt = i - 1; //pegando o indice anterior
                if (lastCIt < 0) {
                  lastCIt = M;
                  while (curveX[cur][lastCIt] < 0)
                    lastCIt--;
                }
                int lastJIt = k - 1;
                if (lastJIt < 0) {
                  lastJIt = M;
                  while (curveX[j][lastJIt] < 0)
                    lastJIt--;
                }
                if (curveX[j][lastJIt] == curveX[cur][lastCIt] && curveY[j][lastJIt] == curveY[cur][lastCIt]) {
                  if (colPlaces[cur][i] < 1) {
                    colPlaces[cur][i] = 1;//colisao no msm sentido
                  }
                }
                else if (curveX[j][lastJIt] != curveX[cur][lastCIt] && curveY[j][lastJIt] != curveY[cur][lastCIt]) {
                  if (colPlaces[cur][i] < 2) {
                    colPlaces[cur][i] = 2;//colisao lateral
                  }
                }
                else {
                  colPlaces[cur][i] = 5;//colisao frontal
                }


              }
            }
          }
        }
      }
    }
    for (int i = 0; i < M; i++) {
      if (colPlaces[cur][i] == 5) {
        int cIt = i;
        int nextCIt = (cIt + 1 != M) ? (cIt + 1) : (0);
        if (colPlaces[cur][nextCIt] == 5) {
          colPlaces[cur][cIt] = 4;
          while (colPlaces[cur][nextCIt] == 5) {
            colPlaces[cur][nextCIt] = 3;
            cIt = nextCIt;
            nextCIt = (cIt + 1 != M) ? (cIt + 1) : (0);
          }
          colPlaces[cur][nextCIt] = 3;
        }

      }
    }
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
    case 0://ir para frente ate quase sair do grid
      if ((getDist(0) > DOTDIST) && !isBlack(0) && !isBlack(3)) {
        setLeftSpeed(0);
        setRightSpeed(0);
        angOff = getAngle();
        begiS = 1;
        if (comm) {
          irCommDeinit();
          comm = !comm;
        }
      }
      else if (getDist(90) > COMDIST || getDist(135) > DOTDIST) {
        setLeftSpeed(0);
        setRightSpeed(0);
        if (!comm) {
          irCommInit();
          comm = !comm;
        }
        if (irCommDataAvailable() == 1) { // && (irCommReceivingSensor() == 0 || irCommReceivingSensor() == 1 || irCommReceivingSensor() == 7)) {
          curve = irCommReadData() - COMMPORTSN;
          upColor();
          irCommSendData(0);
        }
      }
      else {
        if (comm) {
          irCommDeinit();
          comm = !comm;
        }
        follow();
      }
      break;
    case 1://rotating
      if (!rotate(90)) {
        begiS = 2;
        x = 0;
        y = 0;
        if (getSelector() == 5 || curve == N - 1) {
          curve = N - 1;
          upColor();
          begiS = 3;
        }
        curveIt = curveItInit[curve];
      }
      break;
    case -1:
      if (isBlack(1)) {
        setLeftSpeed(-SPEED / 4);
        setRightSpeed(SPEED / 4);
      }
      else
        begiS = 2;
      break;
    case 2://esperando chegar outro
      if (!comm) {
        irCommInit();
        comm = !comm;
      }
      if (getDist(180) > COMDIST) {
        if (irCommDataAvailable() == 1 && 0 == irCommReadData()) { // && (irCommReceivingSensor() == 2 || irCommReceivingSensor() == 1 || irCommReceivingSensor() == 3)) {
          begiS = 3;
          setLeftSpeed(0);
          setRightSpeed(0);
          if (comm) {
            irCommDeinit();
            comm = !comm;
          }
        }
        else {
          irCommSendData(curve + 1 + COMMPORTSN);
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

void cicle() {
  if (curveX[curve][curveIt] < 0)
    curveIt = 0;
  if (!checkCol()) {
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
  }
}

void shake() {
  bool dies = false;
  if (shakeCount > 0) {
    setLeftSpeed(-SPEED / 4);
    setRightSpeed(SPEED / 4);
  }
  else if (shakeCount < 0) {
    setLeftSpeed(SPEED / 4);
    setRightSpeed(-SPEED / 4);
    dies = true;
  }
  if (shakeCount == 0 || (!isBlack(1) && !isBlack(2))) {
    shakeCount = (dies) ? (random(1000)) : (-random(1000));
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
  if (!isBlack(1) && !isBlack(2))
    return;
  if (!isBlack(1)) {
    setRightSpeed(0);
    //printInt(-123);
  }
  if (!isBlack(2)) {
    setLeftSpeed(0);
    //printInt(123);
  }
}

void backward() { //ré
  setLeftSpeed(-SPEED);
  setRightSpeed(-SPEED);
  if (!isBlack(1) && !isBlack(2))
    return;
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
    setLeftSpeed(0);
    setRightSpeed(0);
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
    return true;
  }
  switch (rotationS) {
    case 1:
      if (angle > 0) {
        setLeftSpeed(SPEED );
        setRightSpeed(-SPEED );
      }
      else {
        setLeftSpeed(-SPEED );
        setRightSpeed(SPEED );
      }
      if ((angle > 0) ? (isBlack(0)) : (isBlack(3)))
        rotationS = 2;
      break;
    case 2:
      if ((isBlack(1) || isBlack(2)) && (angle > 0) ? (!isBlack(0)) : (!isBlack(3))) {
        rotationS = 0;
        rotationH = false;
        setLeftSpeed(0);
        setRightSpeed(0);
        globalAngle = (globalAngle + angle + 360) % 360;
        return false;
      }
      else {
        if (angle > 0) {
          setLeftSpeed(SPEED );
          setRightSpeed(-SPEED );
        }
        else {
          setLeftSpeed(-SPEED );
          setRightSpeed(SPEED );
        }
      }
      break;
    case 3:
      if (isBlack(0) && isBlack(3)) {
        rotationS = 4;
      }
      else {
        setLeftSpeed(-SPEED );
        setRightSpeed(-SPEED );
      }
      break;
    case 4:
      if (!isBlack(0)) {
        rotationS = 5;
        setLeftSpeed(0);
        setRightSpeed(0);
      }
      else if (!isBlack(3)) {
        rotationS = 8;
        setLeftSpeed(0);
        setRightSpeed(0);
      }
      else {
        setLeftSpeed(-SPEED );
        setRightSpeed(-SPEED );
      }
      break;
    case 5:
      if (!isBlack(1)) {
        rotationS = 6;
        setLeftSpeed(0);
        setRightSpeed(0);
      }
      else {
        setLeftSpeed(-SPEED );
        setRightSpeed(SPEED );
      }
      break;
    case 6:
      if (!isBlack(2)) {
        rotationS = 7;
        setLeftSpeed(0);
        setRightSpeed(0);
      }
      else {
        setLeftSpeed(-SPEED );
        setRightSpeed(SPEED );
      }
      break;
    case 7:
      if (isBlack(2)) {
        rotationS = 0;
        rotationH = false;
        setLeftSpeed(0);
        setRightSpeed(0);
        globalAngle = (globalAngle + 180) % 360;
        return false;
      }
      else {
        setLeftSpeed(-SPEED );
        setRightSpeed(SPEED );
      }
      break;
    case 8:
      if (!isBlack(2)) {
        rotationS = 9;
        setLeftSpeed(0);
        setRightSpeed(0);
      }
      else {
        setLeftSpeed(SPEED );
        setRightSpeed(-SPEED );
      }
      break;
    case 9:
      if (!isBlack(1)) {
        setLeftSpeed(0);
        setRightSpeed(0);
        rotationS = 10;
      }
      else {
        setLeftSpeed(SPEED );
        setRightSpeed(-SPEED );
      }
      break;
    case 10:
      if (isBlack(1)) {
        rotationS = 0;
        rotationH = false;
        setLeftSpeed(0);
        setRightSpeed(0);
        globalAngle = (globalAngle + 180) % 360;
        return false;
      }
      else {
        setLeftSpeed(SPEED );
        setRightSpeed(-SPEED );
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
  }

  return true;

}
bool checkCol() {
  if (!(rotationH || (translationH && !translationS) )) {
    if ((isBlack(0) && isBlack(3)) && (colPlaces[curve][curveIt] == 3 && frontColStatus < 3)) {
      follow();
      return true;
    }
    switch (colPlaces[curve][curveIt]) {
      case 1:
        if (getDist(90) > COLDIST) {
          setLeftSpeed(0);
          setRightSpeed(0);
          return true;
        }
        break;
      case 2:
        if (getDist(90) > COLDIST) {
          backward();
          return true;
        }
        if ((getDist(45) > COLDIST  || getDist(135) > COLDIST ) &&
            (curvesLen[curve] < curvesLen[colFrom[curve][curveIt]] || (curvesLen[curve] == curvesLen[colFrom[curve][curveIt]] && curve < colFrom[curve][curveIt]))) {
          backward();
          return true;
        }
        break;
      //CASOS 3 e 4 NAO FORAM TESTADOS POIS A COMUNICAÇÂO ENTRE OS ROBOS DEMORA MUITO
      case 3:
        switch (frontColStatus) {
          case 3:
            if (isBlack(0) && isBlack(3)) {
              frontColStatus = 4;
              setLeftSpeed(0);
              setRightSpeed(0);
            }
            else {
              backward();
            }
            return true;
            break;
          case 4:
            if (!isBlack(0) && !isBlack(3)) {
              frontColStatus = 0;
              setLeftSpeed(0);
              setRightSpeed(0);
              translationH = false;
            }
            else {
              follow();
            }
            return true;
            break;
        }
        if (getDist(90) > NEARCOLDIST) {
          if (comm) {
            irCommDeinit();
            comm = !comm;
          }
          backward();
          return true;
        }
        else if (getDist(90) > COMDIST) {
          if (frontColStatus < 3) {
            setLeftSpeed(0);
            setRightSpeed(0);
            if (!comm) {
              irCommInit();
              comm = !comm;
            }
            if (irCommDataAvailable() == 1) { // && (irCommReceivingSensor() == 0 || irCommReceivingSensor() == 1 || irCommReceivingSensor() == 7)) {
              char readTemp = irCommReadData();
              switch (frontColStatus) {
                case 0:
                  if (readTemp >= COMMPORTSN && readTemp < COMMPORTSN + N) {
                    lastCurve = curve;
                    curve = readTemp - COMMPORTSN;
                    upColor();
                    frontColStatus = 1;
                  }
                  break;
                case 1:
                  if (readTemp >= COMMPORTSN + N) {
                    lastCurveIt = curveIt;
                    curveIt = readTemp - COMMPORTSN - N;
                    frontColStatus = 2;
                  }
                  else if (readTemp >= COMMPORTSN && readTemp < COMMPORTSN + N) {
                    missedInfo = true;
                  }
                  break;
                case 2:
                  if (readTemp == 1) {
                    frontColStatus = 3;
                  }
                  else {
                    missedInfo = true;
                  }
                  break;

              }
            }
            switch (frontColStatus) {
              case 0:
                irCommSendData(curve + COMMPORTSN);
                break;
              case 1:
                if (!missedInfo) {
                  irCommSendData(curveIt + N + COMMPORTSN);
                }
                else {
                  irCommSendData(lastCurve + COMMPORTSN);
                  missedInfo = false;
                }
                break;
              case 2:
                if (!missedInfo) {
                  irCommSendData(1);
                }
                else {
                  irCommSendData(lastCurveIt + N + COMMPORTSN);
                  missedInfo = false;
                }
                break;
            }
          }
          else {
            if (comm) {
              irCommDeinit();
              comm = !comm;
            }
          }
          return true;
        }
        else {
          if (comm) {
            irCommDeinit();
            comm = !comm;
            (0);
          }
        }
        break;
      case 4:
        if (getDist(90) > COLDIST) {
          backward();
          return true;
        }
        break;
      //FIM DA PARTE INCOMPLETA
      case 5:
        if (getDist(90) > COLDIST && (curvesLen[curve] < curvesLen[colFrom[curve][curveIt]] || (curvesLen[curve] == curvesLen[colFrom[curve][curveIt]] && curve < colFrom[curve][curveIt]))) {
          backward();
          return true;
        }
        if (getDist(90) > NEARCOLDIST) {
          backward();
          return true;
        }
        break;
    }
  }
  else if ((translationH && !translationS) && getDist(90) > NEARCOLDIST) {
    setLeftSpeed(0);
    setRightSpeed(0);
    return true;
  }
  return false;
}

int getAngle() { // left = 0, ri = 180
  return (900 - currentAngle - angOff) % 360;
}
