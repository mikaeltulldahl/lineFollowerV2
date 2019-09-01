#ifndef LOGGER_H
#define LOGGER_H
#include "Arduino.h"
#include "Linesensor.h"
#include "Positioning.h"
#include <SD.h>

class Logger {
 public:
  Logger(Positioning* posObj, Linesensor* lineObj);
  void init();
  void update(int controllerState);

 private:
  Positioning* positioning;
  Linesensor* linesensor;
  char logFileName[20];
  File logFile;
  bool sdInitialized;
  bool logFileOpen;
};

#endif /* LOGGER_H */