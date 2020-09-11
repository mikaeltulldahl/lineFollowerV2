#ifndef LINESENSOR_H
#define LINESENSOR_H
#include "Arduino.h"
#include "Odometry.h"

class Linesensor {
 public:
  enum SensorState { uninitiated, onLine, lostLineRight, lostLineLeft, inAir };
  volatile SensorState lineSensorState;
  volatile float lineSensorValue;
  volatile float lineHeading;
  volatile float lineSensorOdoX;
  volatile float lineSensorOdoY;
  Linesensor(Odometry* odoObj);
  void init(void);
  void update();

 private:
  Odometry* odometry;
  void measureAll(void);
  void updateCalibration(void);
  void updateLine(void);
  String stateToString(SensorState i);
  void printAsciiLineValue();
  void printAsciiMeasurments();
  void printVolts();
};

#endif /* LINESENSOR_H */