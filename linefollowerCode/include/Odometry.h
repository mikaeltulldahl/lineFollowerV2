#ifndef ODOMETRY_H_
#define ODOMETRY_H_
#include <Arduino.h>

#include "EncoderHelper.h"

class Odometry {
 public:
  Odometry();
  void init();
  void update();
  void calibrateGyroBias();
  void reset();
  float getX();
  float getY();
  int32_t getStepsRight();
  int32_t getStepsLeft();
  float getHeading();
  void resetHeading();
  float getAngVel();
  float getVelocity();
  float getDist();
  volatile boolean gyroCalibrated;

 private:
  volatile float x, y, heading, angVel, velocity;
  volatile int32_t previousTime;
};

#endif /* ODOMETRY_H_ */