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
Logger logger(0);
Linesensor linesensor(0);

volatile int controllerState = 1;  // init = 0, running = 1
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
      case 1:  // init
        if (linesensor.lineSensorState == Linesensor::onLine) {
          controllerState++;
        }
        break;
      case 2:  // reset line calibration, wait to stand still
      if(linesensor.lineSensorState == Linesensor::inAir){
        controllerState = 1;
        Serial.println("return to init");
      }else if (positioning.gyroCalibrated) {
        Serial.println("gyro calibrated");
          positioning.heading = 0;
          controllerState++;
        }
        break;
      case 3:  // turn 360 deg
        if(linesensor.lineSensorState == Linesensor::inAir){
        controllerState = 1;
        Serial.println("return to init");
      }else if (fabs(positioning.heading) >= 360) {
          controllerState++;
        }
        break;
      case 4:  // center on line
        if(linesensor.lineSensorState == Linesensor::inAir){
        controllerState = 1;
        Serial.println("return to init");
      }else if (linesensor.lineSensorState == Linesensor::onLine &&
            fabs(linesensor.lineSensorValue) < 0.01 &&
            fabs(positioning.angVel < 1)) {
          positioning.heading = 0;
          controllerState++;
        }
        break;
      case 5:  // running
        if (linesensor.lineSensorState == Linesensor::inAir ||
            linesensor.lineSensorState == Linesensor::uninitiated) {
                  controllerState = 1;
        Serial.println("return to init");
        }
        break;
        // state: lost line -> if lost more than 200ms, go to init
      default:
                controllerState = 1;
        Serial.println("unknown state");
        break;
    }
    threads.delay(10);
  }
}

void motorControllerThread() {
  float leftPwm;
  float rightPwm;
  volatile float Pomega = 0.01;  // 0.05;
  while (1) {
    positioning.update();
    switch (controllerState) {
      case 1:  // init
      case 2:  // reset line calibration, wait to stand still
        leftPwm = 0;
        rightPwm = 0;
        positioning.calibrateGyroBias();
        break;
      case 3:  // turn 360 deg
      case 4:  // center on line
      case 5:  // running
      default:
        leftPwm = referenceSpeed - Pomega * (positioning.angVel - referenceAngVelRate);
        rightPwm = referenceSpeed + Pomega * (positioning.angVel - referenceAngVelRate);
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
      case 1:  // init
      case 2:  // reset line calibration, wait to stand still
      case 3:  // turn 360 deg
      case 4:  // center on line
        referenceSpeed = 0;
        break;
      case 5:  // running
        referenceSpeed = max(0, 0.2 - 10 * fabs(linesensor.lineSensorValue));
        break;
    }
    threads.delay(2);
  }
}

void angleControllerThread() {
  while (1) {
    logger.update();
    linesensor.update();
    switch (controllerState) {
      case 1:  // init
      case 2:  // reset line calibration, wait to stand still
        referenceAngVelRate = 0;
        break;
      case 3:  // turn 360 deg
        referenceAngVelRate = 180.0f;
        break;
      case 4:  // center on line
      case 5:  // running
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

        float curvature;
        if (absError > length) {
          curvature = 1.0f / length;
        } else {
          // radius = (e^2 + y^2)/(2*e) = 1/curvature
          curvature = (2 * absError) / (absError * absError + length * length);
        }

        // omega = velocity/radius = velocity*curvature;
        float speed = 0.7;
        referenceAngVelRate = 180 / 3.14 * (-speed * curvature * signum);
        //		if (lineSensorState==lostLineLeft){
        //			leftPwm = 0;
        //			rightPwm = -0.5;
        //		}
        //		if (lineSensorState==lostLineRight){
        //			leftPwm = -0.5;
        //			rightPwm = 0;
        //		}
        break;
    }
    threads.delay(2);
  }
}

void setup() {
  digitalWrite(ledPin, HIGH);
  Serial.begin(115200);
  /*while (!Serial && millis() < 3000) {
    ;
  }*/
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
}

void loop() {
  // idle thread
  threads.yield();
}
