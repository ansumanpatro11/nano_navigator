#ifndef globals_h
#define globals_h


#include <stdint.h>
#include "Arduino.h"
#include "Wire.h"
#include <MPU6050_light.h>
#include "src/Deque/Deque.h"

#define north 0
#define east 1
#define south 2
#define west 3

#define wallThreshold 18
// #define leftSensor 0
// #define frontSensor 1
// #define rightSensor 3

#define LEFT_IR A0
#define FRONT_IR A1
#define RIGHT_IR A2

#define MR_ENCA 2 
#define MR_ENCB 4 
#define MR_PWM 5
#define MR_IN2 7
#define MR_IN1 6

#define ML_ENCA 3 
#define ML_ENCB 11 
#define ML_PWM 10
#define ML_IN2 9
#define ML_IN1 8


#define absolute(number) (((number) > 0) ? (number) : -(number))


#define minimum(num1, num2) (((num1) < (num2))? (num1) : (num2))


class IRSensor {
public:
    // Public variables to store the readings
    float left = 0.0;
    float front = 0.0;
    float right = 0.0;

    // Constructor to initialize the pins
    IRSensor(int leftPin, int frontPin, int rightPin)
        : leftPin(leftPin), frontPin(frontPin), rightPin(rightPin) {}

    // Function to update readings
    void updateReadings() {
        left = getDistance(leftPin);
        front = getDistance(frontPin);
        right = getDistance(rightPin);
        delay(50);
    }

private:
    // Private variables to store pin numbers
    int leftPin;
    int frontPin;
    int rightPin;

    // Function to calculate the distance from the sensor reading
    float getDistance(int pin) {
        float voltage = analogRead(pin) * (5.0 / 1024.0); // Convert ADC value to voltage
        return 13.0 / voltage > 30.0 ? 30.0 : 13.0 / voltage; // Calculate distance using the inverse relationship
    }
};

// IRSensor SharpIR(LEFT_IR, FRONT_IR, RIGHT_IR);

class Motor {
public:

    int64_t pos = 0;

    Motor(int ENCA, int ENCB, int PWM, int IN1, int IN2) {
        // Initialize the Motor class with its attributes
        this->ENCA = ENCA;
        this->ENCB = ENCB;
        this->PWM = PWM;
        this->IN1 = IN1;
        this->IN2 = IN2;

        // Set pin modes
        pinMode(ENCA, INPUT_PULLUP);
        pinMode(ENCB, INPUT_PULLUP);
        pinMode(PWM, OUTPUT);
        pinMode(IN1, OUTPUT);
        pinMode(IN2, OUTPUT);
    }

    void drive(int speed)
    {
      if (speed>0) fwd(speed);
      else if (speed == 0) stop();
      else rev(-speed);
    }

    void fwd(int speed)
    {
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(PWM, speed);

    }

    void rev(int speed)
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(PWM, speed);
    }

    void stop()
    {
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, LOW);
      analogWrite(PWM, 0);
    }

private:
    int ENCA;
    int ENCB;
    int PWM;
    int IN1;
    int IN2;
};


class MPUHandler {
public:
    MPUHandler() : mpu(Wire) {}

    void begin() {
        Wire.begin();
        byte status = mpu.begin();
        Serial.print(F("MPU6050 status: "));
        Serial.println(status);
        if (status != 0) {
            Serial.println(F("Could not connect to MPU6050, restarting..."));
            while (1) {} // Stop here; consider implementing a restart mechanism
        }

        Serial.println(F("Calculating offsets, do not move MPU6050"));
        delay(1000);
        mpu.calcOffsets(); // gyro and accelerometer offsets
        Serial.println(F("Done!\n"));
    }

    void update() {
        mpu.update();
    }

    float getAngleZ() {
        return mpu.getAngleZ();
    }

    void resetAngleZ() {
        mpu.resetAngleZ();
    }

private:
    MPU6050 mpu;
};

struct PIDGains {
    float Kp_wall;  // Proportional gain for wall control
    float Kp_diff;  // Proportional gain for difference control
    float Kp_front; // Proportional gain for front control

    // Constructor to initialize all values to zero
    PIDGains() : Kp_wall(5.0), Kp_diff(0.5), Kp_front(10.0) {}


    // Update the PID gains (if needed during runtime)
    void setGains(float new_Kp_wall, float new_Kp_diff, float new_Kp_front) {
      Kp_wall = new_Kp_wall;
      Kp_diff = new_Kp_diff;
      Kp_front = new_Kp_front;
    }
};



// Declare the MPUHandler object
extern MPUHandler mpuHandler;
extern IRSensor SharpIR;
extern Motor rightMotor;
extern Motor leftMotor;
extern PIDGains gains;




struct cell {
  uint8_t neighbours;
  uint8_t visited;
};

extern struct cell PathArray[];

extern Deque dq;

extern uint8_t currentDir;

extern const uint8_t rows, cols;

extern uint8_t startCell, startDir, targetCellsGoal, currentCell,targetCellStart;

extern int sensorValue[];

extern int sensorValueLow[];
extern int sense[];

#endif
