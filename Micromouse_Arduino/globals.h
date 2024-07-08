#ifndef globals_h
#define globals_h

#include <stdint.h>

#define north 0
#define east 1
#define south 2
#define west 3

#define wallThreshold 870
#define leftSensor 0
#define frontSensor 1
#define rightSensor 3


#define absolute(number) (((number) > 0)? (number) : (-(number)))
#define minimum(num1, num2) (((num1) < (num2))? (num1) : (num2))

struct cell {
  uint8_t flood;
  uint8_t neighbours;
  uint8_t visited;
};

extern struct cell floodArray[];

extern const uint8_t rows, cols;

extern uint8_t startCell, startDir, targetCellsGoal[], currentCell,targetCellStart[];

extern int sensorValue[];

#endif