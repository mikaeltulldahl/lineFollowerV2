#ifndef LOGGER_h
#define LOGGER_h
#include "Arduino.h"

class Logger {
 public:
  Logger(int side);
  void init();
  void update();
};

#endif /* LOGGER_h */