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
  volatile float velocity, velocityRight, velocityLeft, distRight, distLeft;
  private:
  volatile int32_t previousTime;
  volatile int32_t oldPosRight, oldPosLeft;
  volatile int32_t microsSinceEncoderMovedRight,microsSinceEncoderMovedLeft;
};

#endif /* POSITIONING_H_ */