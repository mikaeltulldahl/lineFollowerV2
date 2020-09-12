#ifndef ENCODER_HELPER_H
#define ENCODER_HELPER_H
#include <Encoder.h>

#include "Arduino.h"

class EncoderHelper {
 public:
  EncoderHelper(int pin1, int pin2);
  void init();
  float update();
  void reset();
  volatile int32_t dist;
  volatile float distMeters;
  volatile float velocity;

 private:
  Encoder encoder;
  volatile int32_t oldDist;
  volatile int32_t previousTime;
  volatile int32_t microsSinceEncoderMoved;
};

#endif /* ENCODER_HELPER_H */