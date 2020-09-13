#include <Arduino.h>
#include <TeensyThreads.h>

#include "Linesensor.h"
#include "Logger.h"
#include "Motor.h"
#include "Odometry.h"
#include "StateMachine.h"
#include "Utils.h"

volatile uint32_t idleCounter = 0;
int32_t millisSinceCPULoadUpdate = 0;
const int32_t cpuLoadUpdateRate = 5;  // sec
const int ledPin = 13;

Motor rightMotor(0);
Motor leftMotor(1);
Odometry odometry;
Linesensor linesensor(&odometry);
Logger logger(&odometry, &linesensor);

volatile int controllerState = INIT;  // init = 0, running = 1
volatile int32_t runningStartTime = 0;
volatile float referenceAngVelRate = 0;
volatile float referenceSpeed = 0;

const float length = 0.085f;  // mm
#define CALM 1
#if CALM
// nice constants to run calmly
const float Pomega = 0.003f;           // pwm/(deg/sec)
const float Pvel = 3.0f;               // pwm/(meter/sec)
const float runningSpeedMin = -0.05f;  // meter/sec
const float runningSpeedMax = 0.5f;    // meter/sec
const float PslowDown = runningSpeedMax / 0.03f;
const float PslowDown2 = runningSpeedMax / pow2(0.05f);
const float calibrationAngVelRate = 180.0f;  // deg/sec
const float Ptheta2 = 0.0f;
const float Ptheta = 15.0f;
const float angVelMax = 360.0f;
const float startAcc = 1.0f;           // meter/sec^2

#else
// nice constants to run as fast as possible
const float Pomega = 0.003f;         // pwm/(deg/sec)
const float Pvel = 1.2f;             // pwm/(meter/sec)
const float runningSpeedMin = 0.0f;  // meter/sec
const float runningSpeedMax = 1.9f;  // meter/sec
const float PslowDown = 0;
const float PslowDown2 = 20.0f;
const float calibrationAngVelRate = 180.0f;  // deg/sec
const float Ptheta2 = 0.2f;
const float Ptheta = 15.0f;
const float angVelMax = 720.0f;
const float startAcc = 1.0f;           // meter/sec^2
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
        odometry.calibrateGyroBias();
        if (linesensor.lineSensorState == Linesensor::inAir) {
          controllerState = INIT;
          Serial.println("return to init");
        } else if (odometry.gyroCalibrated) {
          Serial.println("gyro calibrated");
          odometry.resetHeading();
          controllerState++;
        }
        break;
      case TURN_360:
        if (linesensor.lineSensorState == Linesensor::inAir) {
          controllerState = INIT;
          Serial.println("return to init");
        } else if (fabs(odometry.getHeading()) >= 360) {
          controllerState++;
        }
        break;
      case CENTER_ON_LINE:
        if (linesensor.lineSensorState == Linesensor::inAir) {
          controllerState = INIT;
          Serial.println("return to init");
        } else if (linesensor.lineSensorState == Linesensor::onLine &&
                   fabs(linesensor.lineSensorValue) < 0.03f &&
                   fabs(odometry.getAngVel()) < 1.0f) {
          odometry.reset();
          controllerState++;
          runningStartTime = millis();
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
    odometry.update();
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
        float speedError = referenceSpeed - odometry.getVelocity();
        float angVelError =
            odometry.getAngVel() -
            max(-angVelMax, min(angVelMax, referenceAngVelRate));
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
        referenceSpeed = 0;
        break;
      case TURN_360:
      case CENTER_ON_LINE:
        referenceSpeed = min(0.1f, max(-0.1f, -1.0f * odometry.getDist()));
        break;
      case RUNNING:
        float slowdown = PslowDown * fabs(linesensor.lineSensorValue);
        float slowdown2 = PslowDown2 * pow2(linesensor.lineSensorValue);
        float runningTime = 0.001f*((float)((int32_t)millis() - runningStartTime));
        float rampedRunningSpeedMax = min(runningSpeedMax, startAcc*runningTime);//limit acceleration at start
        referenceSpeed =
            max(runningSpeedMin, rampedRunningSpeedMax - slowdown - slowdown2);
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
        float angleError = 180 / M_PI * atan(linesensor.lineSensorValue / length);
        float absAngleError = fabs(angleError);
        referenceAngVelRate =
            (Ptheta2 * pow2(absAngleError) + Ptheta * absAngleError) *
            signum(angleError);
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
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);
  Serial.begin(115200);
  delay(5000);
  digitalWrite(ledPin, LOW);
  Serial.println("START");
  threads.setDefaultTimeSlice(1);
  logger.init();
  odometry.init();
  linesensor.init();
  Serial.println("INIT DONE");
  threads.addThread(blinkthread);
  threads.addThread(stateMachinethread);
  threads.addThread(angleControllerThread);
  threads.addThread(speedControllerThread);
  threads.addThread(motorControllerThread);
  threads.addThread(loggerThread,0, 3000);
  threads.addThread(idleThread);
  Serial.println("ADD THREADS DONE");
}

void loop() {
  // idle thread
  threads.yield();
}
