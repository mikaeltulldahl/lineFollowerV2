#ifndef LOGGER_h
#define LOGGER_h
#include "Arduino.h"
#include "Positioning.h"
#include "Linesensor.h"

class Logger {
 public:
  Logger(Positioning *posObj, Linesensor *lineObj);
  void init();
  void update(int controllerState);
  private:
  Positioning *positioning;
  Linesensor *linesensor;
  bool logFileOpen;
};

#endif /* LOGGER_h */