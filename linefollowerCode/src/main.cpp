#include <Arduino.h>
#include <TeensyThreads.h>
#include "Linesensor.h"
#include "Logger.h"
#include "Motor.h"
#include "Positioning.h"

int ledPin = 13;

Motor rightMotor(0);
Motor leftMotor(1);
Positioning positioning(0);
Linesensor linesensor(0);
Logger logger(&positioning, &linesensor);

volatile int controllerState = 1;  // init = 0, running = 1
#define INIT              1
#define LINE_CALIB_RESET  2
#define TURN_360          3
#define CENTER_ON_LINE    4
#define RUNNING           5

volatile float referenceAngVelRate = 0;
volatile float referenceSpeed = 0;

float length = 0.085f;  // mm

void blinkthread() {
  int blinkcode;
  while (1) {
    blinkcode = controllerState;
    for (int i = 0; i < blinkcode; i++) {
      digitalWrite(ledPin, HIGH);
      threads.delay(100);
      digitalWrite(ledPin, LOW);
      threads.delay(100);
    }
    threads.delay(1000);
  }
}

void stateMachinethread() {
  while (1) {
    switch (controllerState) {
      case INIT:
        if (linesensor.lineSensorState == Linesensor::onLine) {
          controllerState++;
        }
        break;
      case LINE_CALIB_RESET:  // wait to stand still
      positioning.calibrateGyroBias();
      if(linesensor.lineSensorState == Linesensor::inAir){
        controllerState = INIT;
        Serial.println("return to init");
      }else if (positioning.gyroCalibrated) {
        Serial.println("gyro calibrated");
          positioning.heading = 0;
          controllerState++;
        }
        break;
      case TURN_360:
        if(linesensor.lineSensorState == Linesensor::inAir){
        controllerState = INIT;
        Serial.println("return to init");
      }else if (fabs(positioning.heading) >= 360) {
          controllerState++;
        }
        break;
      case CENTER_ON_LINE:
        if(linesensor.lineSensorState == Linesensor::inAir){
        controllerState = INIT;
        Serial.println("return to init");
      }else if (linesensor.lineSensorState == Linesensor::onLine &&
            fabs(linesensor.lineSensorValue) < 0.03 &&
            fabs(positioning.angVel < 1)) {
          positioning.reset();
          controllerState++;
        }
        break;
      case RUNNING:
        if (linesensor.lineSensorState == Linesensor::inAir ||
            linesensor.lineSensorState == Linesensor::uninitiated) {
                  controllerState = INIT;
        Serial.println("return to init");
        }
        break;
        // state: lost line -> if lost more than 200ms, go to init
      default:
                controllerState = INIT;
        Serial.println("unknown state");
        break;
    }
    threads.delay(10);
  }
}

void motorControllerThread() {
  float leftPwm;
  float rightPwm;
  volatile float Pomega = 0.003;
  volatile float Pvel = 1.2;
  while (1) {
    positioning.update();
    switch (controllerState) {
      case INIT:
      case LINE_CALIB_RESET:
        leftPwm = 0;
        rightPwm = 0;
        break;
      case TURN_360:
      case CENTER_ON_LINE:
      case RUNNING:
      default:
      float speedError = referenceSpeed - positioning.velocity;
      float angVelError = positioning.angVel - referenceAngVelRate;
        leftPwm = Pvel * speedError - Pomega * angVelError;
        rightPwm = Pvel * speedError + Pomega * angVelError;
        break;
    }
    rightMotor.set(rightPwm);
    leftMotor.set(leftPwm);
    threads.yield();
  }
}

void speedControllerThread() {
  while (1) {
    switch (controllerState) {
      case INIT:
      case LINE_CALIB_RESET:
      case TURN_360:
      case CENTER_ON_LINE:
        referenceSpeed = 0;
        break;
      case RUNNING:
        referenceSpeed = max(0, 1.9 - 20 * linesensor.lineSensorValue*linesensor.lineSensorValue);
        break;
    }
    threads.delay(2);
  }
}

void angleControllerThread() {
  while (1) {
    linesensor.update(positioning.posX, positioning.posY, positioning.heading);
    switch (controllerState) {
      case INIT:
      case LINE_CALIB_RESET:
        referenceAngVelRate = 0;
        break;
      case TURN_360:
        referenceAngVelRate = 180.0f;
        break;
      case CENTER_ON_LINE:
      case RUNNING:
        float error = linesensor.lineSensorValue;
        float absError;
        float signum;
        if (error >= 0) {
          absError = error;
          signum = 1;
        } else {
          absError = -error;
          signum = -1;
        }
        referenceAngVelRate = - (30000*absError*absError + 5000*absError)*max(0.3,positioning.velocity)*signum;
        break;
    }
    threads.delay(1);
  }
}

void loggerThread() {
  while (1) {
    logger.update(controllerState);
    threads.delay(20);
  }
}

void setup() {
  digitalWrite(ledPin, HIGH);
  Serial.begin(115200);
  digitalWrite(ledPin, LOW);
  Serial.println("START");
  threads.setDefaultTimeSlice(1);
  pinMode(ledPin, OUTPUT);
  logger.init();
  positioning.init();
  linesensor.init();
  threads.addThread(blinkthread);
  threads.addThread(stateMachinethread);
  threads.addThread(angleControllerThread);
  threads.addThread(speedControllerThread);
  threads.addThread(motorControllerThread);
  threads.addThread(loggerThread);
}

void loop() {
  // idle thread
  threads.yield();
}
