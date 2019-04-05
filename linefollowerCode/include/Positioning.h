#ifndef POSITIONING_H_
#define POSITIONING_H_
#include <Arduino.h>

class Positioning {
 public:
  Positioning(int i);
  void init();
  void update();
  void calibrateGyroBias();

  volatile float heading, angVel, posX, posY;
  volatile boolean gyroCalibrated;
};

#endif /* POSITIONING_H_ */