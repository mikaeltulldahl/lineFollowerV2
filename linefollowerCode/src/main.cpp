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

volatile int blinkcode = 3;

void blinkthread() {
  while (1) {
    for (int i = 0; i < blinkcode; i++) {
      digitalWrite(ledPin, HIGH);
      threads.delay(150);
      digitalWrite(ledPin, LOW);
      threads.delay(150);
    }
    threads.delay(2000);
    // Serial.println("BLINK");
  }
}

void setup() {
  digitalWrite(ledPin, HIGH);
  Serial.begin(115200);
  while (!Serial && millis() > 10000) {
    ;
  }
  digitalWrite(ledPin, LOW);
  Serial.println("START");
  threads.setDefaultTimeSlice(1);
  pinMode(ledPin, OUTPUT);
  logger.init();
  positioning.init();
  linesensor.init();
  threads.addThread(blinkthread);
}

int controllerState = 0;  // init = 0, running = 1
void loop() {
  positioning.update();
  logger.update();
  linesensor.update();
  threads.delay(1);  // wait for a second

  // update state
  switch (controllerState) {
    case 0:  // init
      if (linesensor.lineSensorState == Linesensor::onLine) {
        controllerState = 1;
      }
      break;
    case 1:  // running
      if (linesensor.lineSensorState == Linesensor::inAir ||
          linesensor.lineSensorState == Linesensor::uninitiated) {
        controllerState = 0;
      }
      break;
  }

  blinkcode = controllerState + 1;
  float leftPwm;
  float rightPwm;

  // act according to state
  switch (controllerState) {
    case 0:  // init
      leftPwm = 0;
      rightPwm = 0;
      break;
    case 1:  // running
    default:
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
      float forwardPwm = 0.3 - 10 * absError;
      if(forwardPwm < 0){
        forwardPwm = 0;
      }

      float length = 0.085f;  // mm
      float curvature;
      if (absError > length) {
        curvature = 1.0f / length;
      } else {
        // radius = (e^2 + y^2)/(2*e) = 1/curvature
        curvature = (2 * absError) / (absError * absError + length * length);
      }

      // omega = velocity/radius = velocity*curvature;
      float speed = 0.7;
      float Pomega = 0.05;
      float referenceAngRate = -speed * curvature * signum;
      leftPwm = forwardPwm - Pomega*(positioning.angVelRad - referenceAngRate);
      rightPwm = forwardPwm + Pomega*(positioning.angVelRad - referenceAngRate);
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

  rightMotor.set(rightPwm);
  leftMotor.set(leftPwm);
}
