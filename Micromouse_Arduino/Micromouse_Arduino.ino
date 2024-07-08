#include "globals.h"
#include "floodfill.h"
#include "save_maze.h"
// #include "Api.h"
const uint8_t rows = 16, cols = 16;

bool flag = 0;
struct cell floodArray[rows * cols];

// uint8_t targetCellsGoal[1] = {12},targetCellStart[1] = {20}, startCell = 20, startDir = 0;
uint8_t targetCellsGoal[4] = {119, 120, 135, 136}, targetCellStart[1] = {248}, startCell = 248, startDir = 0;

int sensorValue[4];

void setup() {
  // Serial.begin(19200);
  // moveForward(1);
  // moveForward(1);
  // turn(-45,90);
  // delay(1000);
  // turn(90,90);



  // log("reset....");
  resetMazeValuesInEEPROM();
  // updateMazeValuesFromEEPROM();
  // log("recalculating....");
  reCalculateFloodValues(targetCellsGoal,4);
  // updateMazeValuesFromEEPROM();

  // log("Starting....");
  initialise();
}


void loop() {
  if(flag==0){


    while (currentCell != targetCellsGoal[0] && currentCell != targetCellsGoal[1] && currentCell != targetCellsGoal[2] && currentCell != targetCellsGoal[3]) {
      updateWalls();
      flood();
      updateTargetCell();
      goToTargetCell();
      floodArray[currentCell].visited = 1;

    }


    reCalculateFloodValues(targetCellStart, 1);
    while (currentCell != targetCellStart[0]) {
      // log(String(currentCell)+" : "+String(floodArray[currentCell].flood));
      updateWalls();
      flood();
      updateTargetCell();
      goToTargetCell();
      floodArray[currentCell].visited = 1;
    }


    reCalculateFloodValues(targetCellsGoal,4);
    while (currentCell != targetCellsGoal[0] && currentCell != targetCellsGoal[1] && currentCell != targetCellsGoal[2] && currentCell != targetCellsGoal[3]) {
      // log(String(currentCell)+" : "+String(floodArray[currentCell].flood));
      updateWalls();
      flood();
      updateTargetCell();
      goToTargetCell();
      floodArray[currentCell].visited = 1;
    }
    flag = 1;
    
  }
  turn(180,90);
}