#ifndef POSITIONING_H_
#define POSITIONING_H_
#include <Arduino.h>
#include "EncoderHelper.h"

class Positioning {
 public:
  Positioning(int i);
  void init();
  void update();
  void calibrateGyroBias();
  void reset();
  float getPosX();
  float getPosY();
  int32_t getDistRight();
  int32_t getDistLeft();
  volatile float heading, angVel;
  volatile boolean gyroCalibrated;
  volatile float velocity;

 private:
  volatile float posX, posY;
  volatile int32_t previousTime;
};

#endif /* POSITIONING_H_ */