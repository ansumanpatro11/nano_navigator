#include "globals.h"
#include "treamux.h"
#include <EEPROM.h>

const uint8_t rows = 12, cols = 5;
// const uint8_t rows = 16, cols = 16;
int pathsize = dq.getSize();
int flag = 0;
int address = 0;

struct cell PathArray[rows * cols];  // Define PathArray

// uint8_t targetCellsGoal[1] = {12},targetCellStart[1] = {20}, startCell = 20, startDir = 0;

uint8_t targetCellsGoal = 2, targetCellStart = 56, startCell = 56, startDir = 0;
// uint8_t targetCellsGoal = 119, targetCellStart = 240, startCell = 240, startDir = 0;

// int sensorValue[4];

// Motor leftMotor(ML_ENCA, ML_ENCB, ML_PWM, ML_IN1, ML_IN2);

void setup() {
  delay(2000);
  Serial.begin(9600);
  pinMode(A3, INPUT);
  // pinMode(13,OUTPUT);
  // //  /hc06.begin(9600);
  // digitalWrite(13, 0);
  // for (int i=0 ;i<4;i++){
  //   sensorValueLow[i]=analogRead(sense[i]);
  // }
  // log("Starting....");
  int x = digitalRead(A3);
  if(EEPROM.read(500) == 100) flag = 2;
  
  attachInterrupt(digitalPinToInterrupt(MR_ENCA), readEncoderRightISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ML_ENCA), readEncoderLeftISR, CHANGE);
  mpuHandler.begin();
  mpuHandler.update();
  initialise();
  moveForward(1);
  delay(200);
}


void loop() {
  if (flag == 0) {
    if (currentCell != targetCellsGoal) {
      updateWalls();
      treamux();

      // log(String(currentCell));
      // updateTargetCell();
      goToTargetCell();
    } else {
      leftMotor.stop();
      rightMotor.stop();
      flag = 1;
      dq.insertRear(currentDir);
      // printDeque();
      // log("i am above");
      pathsize = dq.getSize();
      EEPROM.write(address, pathsize);
      // log("i am below");
      // stop();
      // log(String(flag));
    }
  }

  if (flag == 1) {
    address = 1;
    EEPROM.update(500, 100);
    while (!dq.isEmpty() && address < EEPROM.length()) {

      EEPROM.update(address, dq.getFront());
      dq.deleteFront();
      address++;
    }
    // for (int address = 0; address < pathsize + 1; address++) {
    //   int value = EEPROM.read(address);
    //   log(String(value));
    // }
    // stop();
    flag = 3;
  }
  if (flag == 2) {
    pathsize = EEPROM.read(0);
    for (int address = 2; address < pathsize + 1; address++) {
      // log(String(address));
      int value = EEPROM.read(address);
      // log(String(value));
      getTarget2(value);
      goToTargetCell();
    }
    flag = 4;
    leftMotor.stop();
    rightMotor.stop();
  }
  leftMotor.stop();
  rightMotor.stop();
  EEPROM.update(500, 255);
}
