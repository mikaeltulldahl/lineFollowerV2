#ifndef LINESENSOR_H
#define LINESENSOR_H
#include "Arduino.h"
#include "Positioning.h"

class Linesensor {
 public:
  enum SensorState { uninitiated, onLine, lostLineRight, lostLineLeft, inAir };
  volatile SensorState lineSensorState;
  volatile float lineSensorValue;
  volatile float lineHeading;
  volatile float lineSensorPosX;
  volatile float lineSensorPosY;
  Linesensor(Positioning* posObj);
  void init(void);
  void update();

 private:
  Positioning* positioning;
  void measureAll(void);
  void updateCalibration(void);
  void updateLine(void);
  String stateToString(SensorState i);
  void printAsciiLineValue();
  void printAsciiMeasurments();
  void printVolts();
};

#endif /* LINESENSOR_H */