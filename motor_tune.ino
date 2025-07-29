#include <util/atomic.h> 
#include <SoftwareSerial.h>

#include "Wire.h"
#include <MPU6050_light.h>

#define COUNTS_PER_BLOCK 1150;

SoftwareSerial mySerial(0, 1); // RX, TX

// #define MR_ENCA 2 
// #define MR_ENCB 4 
// #define MR_PWM 5
// #define MR_IN2 7
// #define MR_IN1 6

// #define ML_ENCA 3 
// #define ML_ENCB 11 
// #define ML_PWM 10
// #define ML_IN2 9
// #define ML_IN1 8

// #define LEFT_IR A0
// #define FRONT_IR A1
// #define RIGHT_IR A2

#define TARGET_DISTANCE 3.0


// Define constants
#define WHEEL_DIAMETER 0.038 // in meters (3.8 cm)
#define WHEEL_BASE 0.107     // in meters (10.7 cm)
#define ENCODER_TICKS_PER_METER (1150.0 / 0.3) // Encoder counts per meter
#define KP 3.0               // Proportional gain for P controller

// Encoder data (replace these with actual encoder readings)
volatile int64_t leftEncoderCount = 0;
volatile int64_t rightEncoderCount = 0;



// MPUHandler mpuHandler;
MPUHandler mpuHandler;
IRSensor SharpIR(LEFT_IR, FRONT_IR, RIGHT_IR);

Motor rightMotor(MR_ENCA, MR_ENCB, MR_PWM, MR_IN1, MR_IN2);
Motor leftMotor(ML_ENCA, ML_ENCB, ML_PWM, ML_IN1, ML_IN2);

PIDGains gains;

int getWallError(){
  SharpIR.updateReadings();
  int sideWallError = 0;
  if (SharpIR.left < 20.0 && SharpIR.right < 20.0) {
    sideWallError = gains.Kp_wall * (SharpIR.left - SharpIR.right);
  } 
  // else if(SharpIR.left < 7.0 || SharpIR.right < 7.0){
  //   if(SharpIR.left < 7.0){
  //    sideWallError = gains.Kp_wall * 5 * 2;
  //   }
  //   else{
  //     sideWallError = gains.Kp_wall * 5 * 2;
  //   }
  // }
  else if (SharpIR.left < 20.0) {
    sideWallError = gains.Kp_wall * (SharpIR.left - 10.0);
  } 
  else if (SharpIR.right < 20.0) {
    sideWallError = gains.Kp_wall * (10.0 - SharpIR.right);
  } 
  else {
    sideWallError = 0;
  }
  // Serial.print("sideWallError : ");
  // Serial.println(sideWallError);
  return sideWallError;
}

// ISR
volatile int64_t posiRight = 0;
volatile int64_t posiLeft = 0;

int max_control = 200;
int min_control = 60;
// Define a struct to hold multiple proportional gain constants



void readEncoderRightISR() {
  static bool lastStateB = false;
  bool currentStateA = digitalRead(MR_ENCA);
  bool currentStateB = digitalRead(MR_ENCB);

  // Determine the direction of rotation
  if (currentStateA != lastStateB) {
    posiRight++;
  } else {
    posiRight--;
  }

  lastStateB = currentStateB;
}

void readEncoderLeftISR() {
  static bool lastStateB = false;
  bool currentStateA = digitalRead(ML_ENCA);
  bool currentStateB = digitalRead(ML_ENCB);

  // Determine the direction of rotation
  if (currentStateA != lastStateB) {
    posiLeft++;
  } else {
    posiLeft--;
  }

  lastStateB = currentStateB;
}




class PIDController {
  private:
    float Kp;               // Proportional gain
    float Ki;               // Integral gain
    float Kd;               // Derivative gain
    double integral_error;  // Accumulated error for the integral term
    float prev_error;       // Error from the previous step
    unsigned long long prev_time; // Timestamp of the previous step
    int min_control;        // Minimum control signal (e.g., PWM lower limit)
    int max_control;        // Maximum control signal (e.g., PWM upper limit)

  public:
    // Constructor
    PIDController(float Kp, float Ki, float Kd, int min_control, int max_control)
      : Kp(Kp), Ki(Ki), Kd(Kd), integral_error(0), prev_error(0), prev_time(0),
        min_control(min_control), max_control(max_control) {}

    // Compute the control signal
    float compute(int setpoint, int position) {
      // Get the current time
      unsigned long long current_time = micros();
      
      // Calculate the time elapsed since the prev computation
      float delta_time = (prev_time > 0) ? (current_time - prev_time) / 1.0e6 : 0.0;
      if (prev_time == 0) prev_time = current_time; // Initialize prev_time on first call

      // Calculate the error
      float error = setpoint - position;

      // Update the integral term (only if delta_time is valid)
      if (delta_time > 0) {
        integral_error += error * delta_time;
      }
      // Serial.print("Error :");
      // Serial.println(error);
      // Calculate the derivative term
      float derivative = (delta_time > 0) ? (error - prev_error) / delta_time : 0.0;

      // Compute the control signal
      int control_signal = (Kp * error) + (Ki * integral_error) + (Kd * derivative);
      // Serial.print("signal");
      // Serial.println(control_signal);
      checkLimit(control_signal, min_control, max_control);
      // Serial.print("PWM :");
      // Serial.println(control_signal);
      // Update for the next iteration
      prev_error = error;
      prev_time = current_time;

      return control_signal;
    }

    // Reset the PID state
    void reset() {
      integral_error = 0.0;
      prev_error = 0.0;
      prev_time = 0.0;
    }

    // Update the PID gains (if needed during runtime)
    void setGains(float new_Kp, float new_Ki, float new_Kd) {
      Kp = new_Kp;
      Ki = new_Ki;
      Kd = new_Kd;
    }
};

PIDController rightMotorPID(0.3, 0.0, 0.0, 60.0, 200);
PIDController leftMotorPID(0.3, 0.0, 0.0, 60.0, 200);

void updateEncoderCount() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    rightMotor.pos = -posiRight;
    leftMotor.pos = posiLeft;
  }
}

void alignFront(float distance){
  // Serial.println("Align Front Error : ");
  while(fabs(SharpIR.front - distance) >= 0.5){
    SharpIR.updateReadings();

    int PWM = (SharpIR.front - distance) * gains.Kp_front;
    // Serial.println(SharpIR.front - distance);
    checkLimit(PWM, 60, 200);
    leftMotor.drive(PWM);
    rightMotor.drive(PWM);
  }

  leftMotor.stop();
  rightMotor.stop();
  // Serial.println("Bot Stopped: Target Reached");
}

void checkLimit(int &PWM ,int min_control ,int max_control){
  if (abs(PWM) > max_control) {
    PWM = max_control * (abs(PWM)/PWM);
  } else if (abs(PWM) < min_control) {
    PWM = min_control * (abs(PWM)/PWM);
  }
}


void moveForward(float blocks){
  int target = blocks * COUNTS_PER_BLOCK;

  updateEncoderCount();

  int64_t startPosRight = rightMotor.pos;
  int64_t startPosLeft = leftMotor.pos;

  // float sideWallError = 0.0;
  float fronWallError = 0.0;
  Serial.print(currentCell);
  // Serial.println("Move Forward Error : ");

  while(abs(rightMotor.pos - startPosRight) < target || abs(leftMotor.pos - startPosLeft) < target){
    updateEncoderCount();
    SharpIR.updateReadings();
    // Serial.print("Right : ");
    // Serial.print(abs(rightMotor.pos - startPosRight));
    // Serial.print("  ");
    // Serial.print("Left : ");
    // Serial.print(abs(leftMotor.pos - startPosLeft));
    // Serial.print("  ");

    if(SharpIR.front < 19.0) {
      // fronWallError = (25.0 - (SharpIR.front - 9.0)) * gains.Kp_front;
      alignFront(8.0);
      break;
    }


    // if(SharpIR.front < 7.0) {
    //   leftMotor.stop();
    //   rightMotor.stop();
    //   Serial.println("Bot Stopped: Target Reached");
    //   break;
    // }

    float errorDiff = (rightMotor.pos - startPosRight) - (leftMotor.pos - startPosLeft);
    // Serial.print("Diff Error : ");
    // Serial.print(errorDiff);
    // Serial.print("  ");
    float PWM_EncR = rightMotorPID.compute(target, rightMotor.pos - startPosRight);
    float PWM_EncL = leftMotorPID.compute(target, leftMotor.pos - startPosLeft);

    // Serial.print(rightMotor.pos);
    // Serial.print(" ");
    // Serial.println(target);

    // int PWM_R = PWM_EncR - (errorDiff * gains.Kp_diff);
    // int PWM_L = PWM_EncL + (errorDiff * gains.Kp_diff);

    int sideWallError = getWallError();
    // Serial.print("sideWallError");
    // Serial.print(sideWallError);
    // Serial.print("  ");

    // int PWM_R = PWM_EncR;
    // int PWM_L = PWM_EncL;

    
    // int PWM_R = PWM_EncR + sideWallError;
    // int PWM_L = PWM_EncL - sideWallError;


    int PWM_R = PWM_EncR - (errorDiff * gains.Kp_diff)*0.3 + sideWallError*0.7;
    int PWM_L = PWM_EncL + (errorDiff * gains.Kp_diff)*0.3 - sideWallError*0.7;

    checkLimit(PWM_R, 60, 200);
    checkLimit(PWM_L, 60, 200);

    leftMotor.drive(PWM_L);
    rightMotor.drive(PWM_R);
  }
  Serial.println();
    leftMotor.stop();
    rightMotor.stop();
    leftMotorPID.reset();
    rightMotorPID.reset();
}

float complementaryFilter(float gyroAngle, float encoderAngle, float alpha) {
  // Fuse angles using complementary filter
  return alpha * gyroAngle + (1 - alpha) * encoderAngle;
}

float calculateEncoderAngle() {
  // Calculate wheel displacements
  updateEncoderCount();
  float leftWheelDistance = (float)(leftMotor.pos - leftEncoderCount) / ENCODER_TICKS_PER_METER;
  float rightWheelDistance = (float)(rightMotor.pos - rightEncoderCount) / ENCODER_TICKS_PER_METER;

  // Calculate change in angle based on wheel displacements
  float deltaTheta = (rightWheelDistance - leftWheelDistance) / WHEEL_BASE;

  return deltaTheta * (180 / PI); // Convert radians to degrees
}


void resetEncoders() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    rightEncoderCount = rightMotor.pos;
    leftEncoderCount = leftMotor.pos;
  }
  // leftEncoderCount = 0;
  // rightEncoderCount = 0;
}

void turn(float targetAngle) {
  // Serial.print("TargetAngel: ");
  // Serial.println(targetAngle);
  mpuHandler.update();
  mpuHandler.resetAngleZ(); // Reset the gyro angle Z
  updateEncoderCount();
  resetEncoders();
  float currentAngle = 0;
  // Serial.println("Angle Error : ");
  while (abs(targetAngle - currentAngle) > 5.0) { // Tolerance of 1 degree

    // Serial.println(currentAngle);
    mpuHandler.update();
    float gyroAngleZ = -mpuHandler.getAngleZ();
    float encoderAngleZ = -calculateEncoderAngle();
    // Serial.print("gyroAngleZ Error : ");
    // Serial.print(gyroAngleZ);
    // Serial.print("  ");
    // Serial.println("encoderAngleZ Error : ");
    // Serial.print(encoderAngleZ);
    // Serial.print("  ");
    currentAngle = complementaryFilter(gyroAngleZ, encoderAngleZ, 0.7);
    // Serial.println(currentAngle);
    // Calculate control signal using P controller
    float error = targetAngle - currentAngle;
    // Serial.print("Error : ");
    // Serial.print(error);
    int controlSignal = KP * error;

    // Set motor speeds based on control signal
    // int leftSpeed = constrain(controlSignal, -120, 120);
    checkLimit(controlSignal, 60, 200);
    int rightSpeed = -controlSignal;
    // Serial.print("PWM");
    // Serial.println(rightSpeed);

    // setMotorSpeeds(leftSpeed, rightSpeed);

    leftMotor.drive(controlSignal);
    rightMotor.drive(rightSpeed);

    delay(10); // Small delay to stabilize the loop
  }
  // Serial.println();
  // Stop motors after turning
    leftMotor.drive(0);
    rightMotor.drive(0);
}
// void setup() {
//     Serial.begin(9600);
//     delay(200);
//     mySerial.begin(9600);
//     attachInterrupt(digitalPinToInterrupt(MR_ENCA), readEncoderRightISR, CHANGE);
//     attachInterrupt(digitalPinToInterrupt(ML_ENCA), readEncoderLeftISR, CHANGE);
//     mpuHandler.begin();

//     mpuHandler.update();
//     // turn(180.0);
//     // delay(500);
//     moveForward(1);
//     moveForward(1);
//     turn(90.0);
//     // moveForward(3);
//     // leftMotor.drive(55);
//     // rightMotor.drive(-55);

// }

// void loop() {
  // Check if data is available from the Bluetooth module
  // if (mySerial.available()) {
  //   String data = mySerial.readStringUntil('\n'); // Read data until newline

  //   // Parse the received data
  //   int firstComma = data.indexOf(',');
  //   int secondComma = data.indexOf(',', firstComma + 1);

  //   if (firstComma > 0 && secondComma > 0) {
  //     float kp = data.substring(0, firstComma).toFloat();
  //     float ki = data.substring(firstComma + 1, secondComma).toFloat();
  //     float kd = data.substring(secondComma + 1).toFloat();
  //     leftMotorPID.setGains(kp, ki, kd);
  //     // rightMotorPID.setGains(kp, ki, kd);
  //     gains.Kp_front = kp;
  //   }
  // }
  // int target = COUNTS_PER_BLOCK;

  // updateEncoderCount();

  // int PWM1 = leftMotorPID.compute(target, leftMotor.pos);
  // int PWM2 = rightMotorPID.compute(target, rightMotor.pos);

  // Constrain the control signal within bounds
  // if (fabs(PWM1) > max_control) {
  //   PWM1 = max_control * (fabs(PWM1)/PWM1);
  // } else if (fabs(PWM1) < min_control) {
  //   PWM1 = min_control * (fabs(PWM1)/PWM1);
  // }

  // if (fabs(PWM2) > 120) {
  //   PWM2 = 120 * (fabs(PWM2)/PWM2);
  // } else if (fabs(PWM2) < 30) {
  //   PWM2 = 30 * (fabs(PWM2)/PWM2);
  // }

  // leftMotor.drive(PWM1);
  // rightMotor.drive(PWM2);
  











  // SharpIR.updateReadings();

  // int PWM1 = (SharpIR.front - 10.0) * gains.Kp_front;

  // PWM1 = constrain(PWM1, -200, 200);

  // if (SharpIR.front <= TARGET_DISTANCE + 0.5 && SharpIR.front >= TARGET_DISTANCE - 0.5) {
  //   leftMotor.stop();
  //   rightMotor.stop();
  //   Serial.println("Bot Stopped: Target Reached");
  // } 
  // else {
  //   // Drive motors forward or backward based on control signal
  //   checkLimit(PWM1);
  //   leftMotor.drive(PWM1);
  //   rightMotor.drive(PWM1);
  // }
  // Serial.print(10.0 - SharpIR.front);
  // Serial.print(" ");
  // Serial.print(SharpIR.front);
  // Serial.print(" ");
  // Serial.print(PWM1);
  // Serial.println();


  // leftMotor.drive(200);

  // Serial.println(posiLeft);
    // mpuHandler.update();
    // float gyroAngleZ = mpuHandler.getAngleZ();
    // Serial.println(gyroAngleZ);
    // delay(100); // Optional: Add a delay to control output frequency

  // mpuHandler.update();
// }