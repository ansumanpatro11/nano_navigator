#ifndef API_H
#define API_H

#include <Arduino.h>
// ----- API -----

void log(String message);

int mazeWidth();

int mazeHeight();

bool wallFront();

bool wallRight();

bool wallLeft();

bool moveForward();

void turnRight();

void turnLeft();

void setWall(int x, int y, char direction);

void clearWall(int x, int y, char direction);

void setColor(int x, int y, char color);

void clearColor(int x, int y);

void clearAllColor();

void setText(int x, int y, String text);

void clearText(int x, int y);

void clearAllText();

bool wasReset();

void ackReset();

// ----- Helpers -----

String readline();

String communicate(String command);

bool getAck(String command);

bool getBoolean(String command);

int getInteger(String command);

#endif