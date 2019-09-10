#include <Arduino.h>
#include <TeensyThreads.h>
#include "Linesensor.h"
#include "Logger.h"
#include "Motor.h"
#include "Positioning.h"

volatile uint32_t idleCounter = 0;
int32_t millisSinceCPULoadUpdate = 0;
int32_t cpuLoadUpdateRate = 5;  // sec
const int ledPin = 13;

Motor rightMotor(0);
Motor leftMotor(1);
Positioning positioning;
Linesensor linesensor(&positioning);
Logger logger(&positioning, &linesensor);

volatile int controllerState = 1;  // init = 0, running = 1
#define INIT 1
#define LINE_CALIB_RESET 2
#define TURN_360 3
#define CENTER_ON_LINE 4
#define RUNNING 5

volatile float referenceAngVelRate = 0;
volatile float referenceSpeed = 0;

const float length = 0.085f;  // mm
#define CALM 1
#if CALM
// nice constants to run calmly
const float Pomega = 0.003f;         // pwm/(deg/sec)
const float Pvel = 1.0f;             // pwm/(meter/sec)
const float runningSpeedMin = 0.0f;  // meter/sec
const float runningSpeedMax = 0.15f;  // meter/sec
const float runningSpeedDeviationSlowDown = 20.0f;
const float calibrationAngVelRate = 180.0f;  // deg/sec
const float PthetaSquared = 0.0f;
const float Ptheta = 25000.0f;
#else
// nice constants to run as fast as possible
const float Pomega = 0.003f;         // pwm/(deg/sec)
const float Pvel = 1.2f;             // pwm/(meter/sec)
const float runningSpeedMin = 0.0f;  // meter/sec
const float runningSpeedMax = 1.9f;  // meter/sec
const float runningSpeedDeviationSlowDown = 20.0f;
const float calibrationAngVelRate = 180.0f;  // deg/sec
const float PthetaSquared = 30000.0f;
const float Ptheta = 5000.0f;
#endif

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
        if (linesensor.lineSensorState == Linesensor::inAir) {
          controllerState = INIT;
          Serial.println("return to init");
        } else if (positioning.gyroCalibrated) {
          Serial.println("gyro calibrated");
          positioning.heading = 0;
          controllerState++;
        }
        break;
      case TURN_360:
        if (linesensor.lineSensorState == Linesensor::inAir) {
          controllerState = INIT;
          Serial.println("return to init");
        } else if (fabs(positioning.heading) >= 360) {
          controllerState++;
        }
        break;
      case CENTER_ON_LINE:
        if (linesensor.lineSensorState == Linesensor::inAir) {
          controllerState = INIT;
          Serial.println("return to init");
        } else if (linesensor.lineSensorState == Linesensor::onLine &&
                   fabs(linesensor.lineSensorValue) < 0.03f &&
                   fabs(positioning.angVel) < 1.0f) {
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
        leftPwm = Pvel * speedError + Pomega * angVelError;
        rightPwm = Pvel * speedError - Pomega * angVelError;
        break;
    }
    rightMotor.set(rightPwm);
    leftMotor.set(leftPwm);
    threads.delay(1);
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
        referenceSpeed = max(runningSpeedMin,
                             runningSpeedMax - runningSpeedDeviationSlowDown *
                                                   linesensor.lineSensorValue *
                                                   linesensor.lineSensorValue);
        break;
    }
    threads.delay(2);
  }
}

void angleControllerThread() {
  while (1) {
    linesensor.update();
    switch (controllerState) {
      case INIT:
      case LINE_CALIB_RESET:
        referenceAngVelRate = 0;
        break;
      case TURN_360:
        referenceAngVelRate = calibrationAngVelRate;
        break;
      case CENTER_ON_LINE:
      case RUNNING:
        float error = linesensor.lineSensorValue;
        float absError;
        float signum;
        if (error >= 0.0f) {
          absError = error;
          signum = 1.0f;
        } else {
          absError = -error;
          signum = -1.0f;
        }

        referenceAngVelRate =
            (PthetaSquared * absError * absError + Ptheta * absError) *
            max(0.3f, positioning.velocity) * signum;
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

void idleThread() {
  while (1) {
    int32_t dt = (int32_t)millis() - millisSinceCPULoadUpdate;
    if (dt >= 1000 * cpuLoadUpdateRate) {
      Serial.print("free cpu: ");
      Serial.print(idleCounter / 100);
      Serial.println("%");
      idleCounter = 0;
      millisSinceCPULoadUpdate += dt;
    }

    // do some pointless heavy lifting to burn cpu cycles;
    for (volatile int i = 0; i < 311 * cpuLoadUpdateRate; i++) {
      volatile float a = 123;
      volatile float b = sqrtf(a);
      (void)b;  // to suppress [-Wunused-variable] warning
    }
    idleCounter++;
    threads.yield();
  }
}

void setup() {
  digitalWrite(ledPin, HIGH);
  Serial.begin(115200);
  delay(5000);
  digitalWrite(ledPin, LOW);
  Serial.println("START");
  threads.setDefaultTimeSlice(1);
  pinMode(ledPin, OUTPUT);
  logger.init();
  positioning.init();
  linesensor.init();
  Serial.println("INIT DONE");
  threads.addThread(blinkthread);
  threads.addThread(stateMachinethread);
  threads.addThread(angleControllerThread);
  threads.addThread(speedControllerThread);
  threads.addThread(motorControllerThread);
  threads.addThread(loggerThread);
  threads.addThread(idleThread);
}

void loop() {
  // idle thread
  threads.yield();
}
