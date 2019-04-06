#ifndef LINESENSOR_H
#define LINESENSOR_H
#include "Arduino.h"

class Linesensor {
 public:
  enum SensorState { uninitiated, onLine, lostLineRight, lostLineLeft, inAir };
  volatile SensorState lineSensorState;
  volatile float lineSensorValue;
  volatile float lineHeading;
  Linesensor(int side);
  void init(void);
  void update(volatile float posX, volatile float posY, volatile float heading);

 private:
  void measureAll(void);
  void updateCalibration(void);
  void updateLine(void);
  String stateToString(SensorState i);
  volatile float lineSensorPosX;
  volatile float lineSensorPosY;
};

#endif /* LINESENSOR_H */