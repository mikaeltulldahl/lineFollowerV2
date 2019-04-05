#ifndef LINESENSOR_H
#define LINESENSOR_H
#include "Arduino.h"

class Linesensor {
 public:
  enum SensorState { uninitiated, onLine, lostLineRight, lostLineLeft, inAir };
  volatile SensorState lineSensorState;
  volatile float lineSensorValue;
  Linesensor(int side);
  void init(void);
  void update();

 private:
  void measureAll(void);
  void updateCalibration(void);
  void updateLine(void);
  String stateToString(SensorState i);
};

#endif /* LINESENSOR_H */