#ifndef LOGGER_h
#define LOGGER_h
#include "Arduino.h"
#include "Linesensor.h"
#include "Positioning.h"

class Logger {
 public:
  Logger(Positioning* posObj, Linesensor* lineObj);
  void init();
  void update(int controllerState);

 private:
  Positioning* positioning;
  Linesensor* linesensor;
  bool logFileOpen;
};

#endif /* LOGGER_h */