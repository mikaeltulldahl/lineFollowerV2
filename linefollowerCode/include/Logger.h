#ifndef LOGGER_H
#define LOGGER_H
#include <SD.h>

#include "Arduino.h"
#include "Linesensor.h"
#include "Odometry.h"

class Logger {
 public:
  Logger(Odometry* odoObj, Linesensor* lineObj);
  void init();
  void update(int controllerState);

 private:
  Odometry* odometry;
  Linesensor* linesensor;
  char logFileName[20];
  File logFile;
  bool sdInitialized;
  bool logFileOpen;
};

#endif /* LOGGER_H */